# Copyright (c) 2026 wee733
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Robot-description helpers for the ARX R5A."""

import xacro


def get_robot_description_contents(
    urdf_path: str,
    initial_positions_file_path: str,
) -> str:
    """Process the real-robot xacro and return its XML contents."""
    document = xacro.process_file(
        urdf_path,
        mappings={
            'initial_positions_file': initial_positions_file_path,
        },
    )
    return document.toxml()
