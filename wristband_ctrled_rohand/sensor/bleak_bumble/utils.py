# SPDX-License-Identifier: MIT
# Copyright (c) 2024 Victor Chavez <vchavezb@protonmail.com>
import bumble.core
from bleak.uuids import normalize_uuid_str


def bumble_uuid_to_str(bumble_uuid: bumble.core.UUID) -> str:
    """
    Converts a native Bumble UUID to a standard string representation.
    Bumble's string representation (`__str__`) is non-standard, and its byte representation
    (`uuid_bytes`) is in reverse order. This function corrects these issues to provide a
    consistent and standard UUID string format.

    Example for a Bumble shortened 16-bit UUID:
        Bumble UUID string representation: 'UUID-16:1800 (Generic Access)'
        Bumble UUID bytes representation: b'\x00\x18'

    Args:
        uuid (UUID): The Bumble UUID object to be converted.

    Returns:
        str: The standard string representation of the UUID.
    """
    # Reverse Bumble's little-endian byte representation
    uuid_bytes = bumble_uuid.uuid_bytes[::-1]
    raw_str = "".join(f"{b:02x}" for b in uuid_bytes)
    return normalize_uuid_str(raw_str)
