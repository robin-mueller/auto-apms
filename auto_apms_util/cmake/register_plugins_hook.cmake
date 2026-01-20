# Copyright 2024 Robin Müller
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(DEFINED _AUTO_APMS_UTIL__PLUGINS_XML_CONTENT)
  file(GENERATE
    OUTPUT "${_AUTO_APMS_UTIL__BUILD_DIR_ABSOLUTE}/${PROJECT_NAME}_plugins.xml"
    CONTENT "<class_libraries>\n${_AUTO_APMS_UTIL__PLUGINS_XML_CONTENT}</class_libraries>"
  )
  install(
    FILES
    "${_AUTO_APMS_UTIL__BUILD_DIR_ABSOLUTE}/${PROJECT_NAME}_plugins.xml"
    DESTINATION
    "${_AUTO_APMS_UTIL__AUTO_APMS_SHARED_RESOURCES_DIR_RELATIVE}"
  )
  ament_index_register_resource(
    "${_AUTO_APMS_UTIL__RESOURCE_TYPE_NAME__PLUGINLIB}"
    CONTENT
    "${_AUTO_APMS_UTIL__AUTO_APMS_SHARED_RESOURCES_DIR_RELATIVE}/${PROJECT_NAME}_plugins.xml"
  )
endif()

