# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def create_square_from_center(center_x, center_y, size):
    """
    Create a square centered at (`center_x`, `center_y`) with side length `size`.
    """
    half_size = size / 2
    return [
        (center_x - half_size, center_y - half_size),  # Bottom-left
        (center_x - half_size, center_y + half_size),  # Top-left
        (center_x + half_size, center_y + half_size),  # Top-right
        (center_x + half_size, center_y - half_size),  # Bottom-right
    ]
