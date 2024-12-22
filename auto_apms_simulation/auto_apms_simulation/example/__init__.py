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

from .hogwarts import HogwartsWorld

_WORLDS = [HogwartsWorld]


def get_world_names():
    return [w.NAME for w in _WORLDS]


def create_world_from_name(name: str, **kwargs):
    for world in _WORLDS:
        if name.casefold() == world.NAME.casefold():
            return world(**kwargs)
    raise ValueError(f"World '{name}' does not exist.")
