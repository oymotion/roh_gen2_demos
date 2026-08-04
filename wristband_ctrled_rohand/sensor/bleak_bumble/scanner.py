# SPDX-License-Identifier: MIT
# Copyright (c) 2024 Victor Chavez <vchavezb@protonmail.com>

import logging
import sys
from asyncio import sleep
from typing import Dict, Final, List, Literal, Optional, Tuple

from bleak.backends.scanner import (
    AdvertisementData,
    AdvertisementDataCallback,
    BaseBleakScanner,
)
from bumble.controller import Controller
from bumble.core import UUID, AdvertisingData
from bumble.device import Advertisement, Device, DeviceConfiguration
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
    from typing_extensions import override as override
else:
    from typing import override as override

logger = logging.getLogger(__name__)

SERVICE_UUID_TYPES: Final[Tuple] = (
    AdvertisingData.COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    AdvertisingData.COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
    AdvertisingData.COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS,
    AdvertisingData.INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS,
    AdvertisingData.INCOMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS,
    AdvertisingData.INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
)

# Arbitrary BD_ADDR for the scanner device
# SCANNER_BD_ADDR = "00:00:00:00:00:00"
# A static BD_ADDR. Use with suffix '/P' for public (fixed) address.
SCANNER_BD_ADDR = "F0:F1:F2:F3:F4:F5"


def get_local_name(adv: Advertisement) -> str:
    """
    Get the local name from an advertisement object.
    Local name is not always present in the advertisement data,
    see Bluetooth Core Specification 6.0, Vol 3, Part C, Section 9.2.5.2 Conditions.
    If not found then return an empty string.
    :param adv: Advertisement object.
    return: Local name string.
    """
    local_name = ""
    adv_data = adv.data.get(AdvertisingData.COMPLETE_LOCAL_NAME)
    if adv_data is None or adv_data == "":
        adv_data = adv.data.get(AdvertisingData.SHORTENED_LOCAL_NAME)
    if isinstance(adv_data, str):
        local_name = adv_data
    return local_name


def get_manuf_data(adv: Advertisement) -> Dict[int, bytes]:
    """
    Get the manufacturer data from an advertisement object.
    :param adv: Advertisement object.
    return: Manufacturer data dictionary.
    """
    manuf_data = {}
    adv_data = adv.data.get(AdvertisingData.MANUFACTURER_SPECIFIC_DATA)
    # Manufacturer data is a tuple of company ID and data,
    # see bumble.core.ad_data_to_object
    if isinstance(adv_data, tuple) and len(adv_data) == 2:
        company_id, data = adv_data
        if isinstance(company_id, int) and isinstance(data, bytes):
            manuf_data[company_id] = data
    return manuf_data


def get_service_data(adv: Advertisement) -> Dict[str, bytes]:
    """
    Get the service data from a bumble advertisement object to
    the bleak expected format.
    :param adv: Advertisement object.
    return: Service data dictionary.
    """
    service_data = {}
    adv_data = adv.data.get(AdvertisingData.SERVICE_DATA)
    # Bumble manufacturer data is a tuple of UUID and bytes,
    # see bumble.core.ad_data_to_object
    if isinstance(adv_data, tuple) and len(adv_data) == 2:
        uuid, data = adv_data
        if isinstance(uuid, UUID) and isinstance(data, bytes):
            service_data[bumble_uuid_to_str(uuid)] = data
    return service_data


def get_service_uuids(adv: Advertisement) -> List[str]:
    """
    Get the service UUIDs from an advertisement object.
    :param adv: Advertisement object.
    return: List of service UUID strings.
    """
    service_uuids = []
    for service_uuid_type in SERVICE_UUID_TYPES:
        adv_data = adv.data.get(service_uuid_type)
        # Bumble service uuids is a list of UUIDs,
        # see bumble.core.ad_data_to_object
        if isinstance(adv_data, list):
            for uuid in adv_data:
                if isinstance(uuid, UUID):
                    service_uuids.append(bumble_uuid_to_str(uuid))
    return service_uuids


class BleakScannerBumble(BaseBleakScanner):
    """
    Interface for Bleak Bluetooth LE Scanners

    Args:
        detection_callback:
            Optional function that will be called each time a device is
            discovered or advertising data has changed.
        service_uuids:
            Optional list of service UUIDs to filter on. Only advertisements
            containing this advertising data will be received.
        scanning_mode:
            Set to ``"passive"`` to avoid the ``"active"`` scanning mode.

    Keyword Args:
        cfg: Bumble transport configuration.
        host_mode:
            Set to ``True`` to set bumble as an HCI Host. Useful
            for connecting an external HCI controller
            If ``False`` it will be set as a controller.
        dev_cfg:
            Optional DeviceConfiguration
    """

    def __init__(
        self,
        detection_callback: Optional[AdvertisementDataCallback],
        service_uuids: Optional[List[str]],
        scanning_mode: Literal["active", "passive"],
        **kwargs,
    ):
        super().__init__(detection_callback, service_uuids)

        self._dev: Optional[Device] = None
        self._scan_active: Final = scanning_mode == "active"
        self._cfg: Final[BumbleTransportCfg] = kwargs.get(
            "cfg", get_default_transport_cfg()
        )
        self._host_mode: Final[bool] = kwargs.get(
            "host_mode", is_host_mode_enabled_from_env()
        )

        # Device configuration
        self._dev_cfg: DeviceConfiguration = kwargs.get(
            "dev_cfg",
            DeviceConfiguration.from_dict(
                {"name": "scanner", "address": SCANNER_BD_ADDR}
            ),
        )

    def on_advertisement(self, advertisement: Advertisement):
        local_name = get_local_name(advertisement)
        if not self.is_allowed_uuid(uuids := get_service_uuids(advertisement)):
            return
        advertisement_data = AdvertisementData(
            local_name=local_name,
            manufacturer_data=get_manuf_data(advertisement),
            service_data=get_service_data(advertisement),
            service_uuids=uuids,
            tx_power=advertisement.tx_power,
            rssi=advertisement.rssi,
            platform_data=(None, None),
        )
        addr = str(advertisement.address)
        device = self.create_or_update_device(
            key=addr,
            address=addr,
            name=local_name,
            details={},
            adv=advertisement_data,
        )
        self.call_detection_callbacks(device, advertisement_data)

    async def on_connection(self, connection):
        pass

    @override
    async def start(self) -> None:
        transport = await start_transport(self._cfg, self._host_mode)
        if not self._host_mode:
            self._dev = Device(config=self._dev_cfg)
            self._dev.host = Host()
            self._dev.host.controller = Controller("scanner", link=get_link())
        else:
            self._dev = Device.from_config_with_hci(
                self._dev_cfg, transport.source, transport.sink
            )
        self._dev.on("advertisement", self.on_advertisement)
        await self._dev.power_on()
        await self._dev.start_scanning(active=self._scan_active)

    @override
    async def stop(self) -> None:
        if self._dev is None:
            logger.debug("Scanner not started.")
            return
        await self._dev.stop_scanning()
        # The transport must be closed in host_mode.
        if self._host_mode:
            transport = transports.pop(str(self._cfg), None)
            if transport is not None:
                await transport.close()
        await self._dev.power_off()
        await sleep(1)  # Wait for stabilization.
        self._dev = None

    def set_scanning_filter(self, **kwargs) -> None:
        """Implement scanning filter setup

        Raises:
            NotImplementedError:
            This is not implemented yet.

        """
        raise NotImplementedError("set_scanning_filter is not implemented yet.")
