# SPDX-License-Identifier: MIT
# Copyright (c) 2024 Victor Chavez <vchavezb@protonmail.com>
"""
BLE Client for Bumble
"""

import logging
import sys
import warnings
from asyncio import sleep
from functools import partial
from typing import Dict, Final, List, Optional, Union

from bleak import BleakGATTDescriptor
from bleak.assigned_numbers import gatt_char_props_to_strs
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.client import BaseBleakClient, NotifyCallback
from bleak.backends.device import BLEDevice
from bleak.backends.service import BleakGATTService, BleakGATTServiceCollection
from bleak.exc import BleakDeviceNotFoundError, BleakError
from bumble.controller import Controller
from bumble.core import UUID, TimeoutError
from bumble.device import (
    Connection,
    ConnectionParametersPreferences,
    Device,
    DeviceConfiguration,
    Peer,
)
from bumble.hci import HCI_LE_1M_PHY, HCI_LE_2M_PHY, HCI_LE_CODED_PHY, Phy
from bumble.host import Host

from sensor.bleak_bumble import (
    BumbleTransportCfg,
    get_default_transport_cfg,
    get_link,
    is_host_mode_enabled_from_env,
    start_transport,
    transports,
)
from sensor.bleak_bumble.utils import bumble_uuid_to_str

if sys.version_info < (3, 12):
    from typing_extensions import Buffer
    from typing_extensions import override as override
else:
    from collections.abc import Buffer
    from typing import override as override

# A static BD_ADDR. Use with suffix '/P' for public (fixed) address.
CLIENT_BD_ADDR = "F0:F1:F2:F3:F4:F5"


logger = logging.getLogger(__name__)


class BleakClientBumble(BaseBleakClient):
    """Bumble class interface for BleakClient

    Args:
        address_or_ble_device: The Bluetooth address of the
            BLE peripheral to connect to or the BLEDevice object representing it.
    Keyword Args:
        cfg: Bumble transport configuration.
        host_mode:
            Set to ``True`` to set bumble as an HCI Host. Useful
            for connecting an external HCI controller
            If ``False`` it will be set as a controller.
        phys:
            Set a comma separated string consisting of '1m', '2m' and 'coded'.
            Preferences for the default 1M PHY are always set by the backend.
        dev_cfg:
            Optional DeviceConfiguration

    """

    def __init__(self, address_or_ble_device: Union[BLEDevice, str], **kwargs):
        super().__init__(address_or_ble_device, pair=False, **kwargs)
        self._peer: Optional[Peer] = None
        self._dev: Optional[Device] = None
        self._connection: Optional[Connection] = None
        self._subs: Dict[int, list[NotifyCallback]] = {}
        self._cfg: Final[BumbleTransportCfg] = kwargs.get(
            "cfg", get_default_transport_cfg()
        )
        self._host_mode: Final[bool] = kwargs.get(
            "host_mode", is_host_mode_enabled_from_env()
        )

        # Parse PHYs.
        self._phys: Optional[List[Phy]] = None
        phys: Final[str | None] = kwargs.get("phys", None)
        if phys is not None:  # pragma: no cover
            self._phys = []
            elements = phys.lower().split(",")
            for element in elements:
                if element == "1m":
                    self._phys.append(HCI_LE_1M_PHY)
                elif element == "2m":
                    self._phys.append(HCI_LE_2M_PHY)
                elif element == "coded":
                    self._phys.append(HCI_LE_CODED_PHY)
                else:
                    raise ValueError("invalid PHY name")
        self._connection_parameters_preferences: (
            Dict[Phy, ConnectionParametersPreferences] | None
        ) = None

        # Device configuration
        self._dev_cfg: DeviceConfiguration = kwargs.get(
            "dev_cfg",
            DeviceConfiguration.from_dict(
                {"name": "client", "address": CLIENT_BD_ADDR}
            ),
        )

        # Use stored peer name in BLEDevice if exists
        self._name: str = ""
        if isinstance(address_or_ble_device, BLEDevice) and address_or_ble_device.name:
            self._name = str(address_or_ble_device.name)

    @property
    @override
    def mtu_size(self) -> int:
        """Get ATT MTU size for active connection."""
        if not self._connection:
            raise BleakError("Not connected")
        return self._connection.att_mtu

    @override
    async def connect(self, pair: bool = False, **kwargs) -> None:
        """Connect to the specified GATT server.

        Returns:
            None.

        """
        if pair:
            logger.debug(
                "Pair with the peripheral before connecting is not implemented."
            )

        timeout = self._timeout
        transport = await start_transport(self._cfg, self._host_mode)
        if not self._host_mode:
            self._dev = Device(config=self._dev_cfg)
            self._dev.host = Host()
            self._dev.host.controller = Controller("Client", link=get_link())
        else:
            self._dev = Device.from_config_with_hci(
                self._dev_cfg, transport.source, transport.sink
            )
        self._dev.on("connection", self.on_connection)
        logger.debug("Connecting to device @ %s", self.address)
        if self._dev.is_scanning:
            await self._dev.stop_scanning()
        await self._dev.power_on()

        # Set up PHYs and their ConnectionParametersPreferences.
        if self._phys is not None:  # pragma: no cover
            if self._connection_parameters_preferences is None:
                self._connection_parameters_preferences = {
                    phy: ConnectionParametersPreferences.default for phy in self._phys
                }
            await self._dev.set_default_phy(tx_phys=self._phys, rx_phys=self._phys)
        else:
            self._connection_parameters_preferences = None

        try:
            await self._dev.connect(
                self.address,
                connection_parameters_preferences=self._connection_parameters_preferences,
                timeout=timeout,
            )
        except TimeoutError:
            # The transport must be closed in host_mode.
            if self._host_mode:
                await self._close_transport()
            logger.debug("Connection timed out")
            raise BleakDeviceNotFoundError("The device was not found.")

        if self._connection:
            self.services: BleakGATTServiceCollection = await self.get_services()
            self._name = await self._get_peer_name()
        return None

    @override
    async def disconnect(self) -> None:
        """Disconnect from the specified GATT server.

        Returns:
            None.

        """
        await sleep(1)  # Avoid race condition with the delay.
        logger.debug("Disconnecting from BLE device")
        if self._dev and self._connection:
            await self._connection.disconnect()
        else:
            logger.debug("already disconnected")

        # The transport must be closed in host_mode.
        if self._host_mode:
            await self._close_transport()
        return None

    async def _close_transport(self) -> None:
        transport = transports.pop(str(self._cfg), None)
        if transport is not None:
            await transport.close()
            await sleep(1)  # Wait for stabilization.
        self._dev = None

    @override
    async def pair(self, *args, **kwargs) -> None:
        """Pair with the peripheral."""
        if self._peer:
            await self._peer.connection.pair()
        return None

    @override
    async def unpair(self) -> None:
        """Unpair with the peripheral."""
        if hasattr(self._dev, "keystore"):
            try:
                await self._dev.keystore.delete(str(self._connection.peer_address))  # type: ignore  # (missing type hints in bumble)
            except:
                logger.debug("Device not found or already unpaired.")
        return None

    @property
    @override
    def is_connected(self) -> bool:
        """Check connection status between this client and the server.

        Returns:
            Boolean representing connection status.

        """
        return bool(self._connection)

    @property
    @override
    def name(self) -> str:
        """See :meth:`bleak.BleakClient.name`."""
        if not self._peer:
            raise BleakError("Not connected")
        return self._name

    async def _get_peer_name(self) -> str:
        """A helper function to obtain the name

        `self._name`, obtained before connection
        if `isinstance(address_or_ble_device, BLEDevice) and address_or_ble_device.name`,
        would be updated by reading `Device Name` characteristic.

        A formatted BLE address would be used if, at all, failed in obtaining the name.

        Returns:
            (str) The peer name or a formatted BLE address of the peer.

        """
        if not self._peer:
            raise BleakError("Not connected")

        # Read Device Name in Generic Access Service
        name = ""
        try:
            char_vals = await self._peer.read_characteristics_by_uuid(
                UUID("00002a00-0000-1000-8000-00805f9b34fb")
            )
            name = char_vals[0].decode("utf8")
        except:
            pass
        # If Device Name is not available, use peer address.
        if not (self._name or name):
            name = self._peer.connection.peer_address.to_string(
                with_type_qualifier=False
            ).replace(":", "-")

        return name if name else self._name

    async def get_services(self, **kwargs) -> BleakGATTServiceCollection:
        """Get all services registered for this GATT server.

        Returns:
           A :py:class:`bleak.backends.service.BleakGATTServiceCollection` with this device's services tree.

        """
        if not self._connection:
            raise BleakError("Not connected")

        if self.services is not None:
            return self.services

        new_services = BleakGATTServiceCollection()
        self._peer = Peer(self._connection)
        await self._peer.discover_services()
        for service in self._peer.services:
            bleak_svc = BleakGATTService(
                obj=service,
                handle=service.handle,
                uuid=bumble_uuid_to_str(service.uuid),
            )
            new_services.add_service(bleak_svc)
            await service.discover_characteristics()
            for characteristic in service.characteristics:
                await characteristic.discover_descriptors()
                bleak_char = BleakGATTCharacteristic(
                    obj=characteristic,
                    handle=characteristic.handle,
                    uuid=bumble_uuid_to_str(characteristic.uuid),
                    properties=gatt_char_props_to_strs(characteristic.properties),
                    max_write_without_response_size=lambda: self.mtu_size - 3,
                    service=bleak_svc,
                )
                new_services.add_characteristic(bleak_char)
                for descr in characteristic.descriptors:
                    bleak_dscr = BleakGATTDescriptor(
                        obj=descr,
                        handle=descr.handle,
                        uuid=bumble_uuid_to_str(descr.type),
                        characteristic=bleak_char,
                    )
                    new_services.add_descriptor(bleak_dscr)

        return new_services

    @override
    async def read_gatt_char(
        self,
        characteristic: BleakGATTCharacteristic,
        **kwargs,
    ) -> bytearray:
        """Perform read operation on the specified GATT characteristic.

        Args:
            characteristic (BleakGATTCharacteristic): The characteristic to read from.

        Returns:
            (bytearray) The read data.

        """
        if self._peer is None:
            raise BleakError("Not connected")

        char_vals = await self._peer.read_characteristics_by_uuid(
            characteristic.obj.uuid
        )
        value = char_vals[0]
        logger.debug(f"Read Characteristic {characteristic} : {value.hex()}")
        return bytearray(value)

    @override
    async def read_gatt_descriptor(
        self,
        descriptor: BleakGATTDescriptor,
        **kwargs,
    ) -> bytearray:
        """Perform read operation on the specified GATT descriptor.

        Args:
            descriptor: The descriptor to read from.

        Returns:
            (bytearray) The read data.

        """
        if not self.is_connected:
            raise BleakError("Not connected")

        val = await descriptor.obj.read_value()
        logger.debug(f"Read Descriptor {descriptor} : {val}")
        return val

    @override
    async def write_gatt_char(
        self,
        characteristic: BleakGATTCharacteristic,
        data: Buffer,
        response: bool,
    ) -> None:
        """
        Perform a write operation on the specified GATT characteristic.

        Args:
            characteristic: The characteristic to write to.
            data: The data to send.
            response: If write-with-response operation should be done.
        """
        if not self.is_connected:
            raise BleakError("Not connected")

        await characteristic.obj.write_value(data, with_response=response)
        logger.debug(f"Write Characteristic {characteristic.uuid} : {data}")

    @override
    async def write_gatt_descriptor(
        self, descriptor: BleakGATTDescriptor, data: Buffer
    ) -> None:
        """Perform a write operation on the specified GATT descriptor.

        Args:
            descriptor: The descriptor to write to.
            data: The data to send (any bytes-like object).

        """
        if not self.is_connected:
            raise BleakError("Not connected")

        await descriptor.obj.write_value(data)
        logger.debug(f"Write Descriptor {descriptor} : {data}")

    def __notify_handler(self, characteristic: BleakGATTCharacteristic, value):
        for sub in self._subs[characteristic.handle]:
            sub(value)

    @override
    async def start_notify(
        self,
        characteristic: BleakGATTCharacteristic,
        callback: NotifyCallback,
        **kwargs,
    ) -> None:
        """
        Activate notifications/indications on a characteristic.

        Keyword Args:
            force_indicate (bool): If this is set to True, then Bleak will set up
                a indication request instead of a notification request, given that
                the characteristic supports notifications as well as indications.

        Implementers should call the OS function to enable notifications or
        indications on the characteristic.

        To keep things the same cross-platform, notifications should be preferred
        over indications if possible when a characteristic supports both.
        """
        if not self.is_connected:
            raise BleakError("Not connected")

        prefer_notify = not kwargs.get("force_indicate", False)
        if not self._subs.get(characteristic.handle):
            self._subs[characteristic.handle] = []
        self._subs[characteristic.handle].append(callback)
        await characteristic.obj.subscribe(
            partial(self.__notify_handler, characteristic),
            prefer_notify=prefer_notify,
        )

    @override
    async def stop_notify(
        self,
        characteristic: BleakGATTCharacteristic,
    ) -> None:
        """Deactivate notification/indication on a specified characteristic.

        Args:
            characteristic (BleakGATTCharacteristic): The characteristic to deactivate
                notification/indication on.

        """
        if not self.is_connected:
            raise BleakError("Not connected")

        await characteristic.obj.unsubscribe()
        self._subs.pop(characteristic.handle, None)

    def on_connection(self, connection: Connection):
        self._connection = connection
        self._connection.on("disconnection", self.on_disconnection)
        self._subs = {}

    def on_disconnection(self, reason):
        self._connection = None
        if self._disconnected_callback:
            self._disconnected_callback()
        self._subs = {}
