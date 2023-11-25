import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export async function createPackage() {
    // Package name
    const packageName = await vscode.window.showInputBox({
        prompt: 'Enter the name for your ROS2 package'
    });

    if (!packageName) {
        vscode.window.showErrorMessage('Package name is required.');
        return;
    }

    // Package slug (package_name)
    const packageSlug = packageName.toLowerCase().replace(/ /g, "_");

    // Package description
    const packageDescription = await vscode.window.showInputBox({
        prompt: 'Enter the description of the package'
    });

    if (!packageDescription) {
        vscode.window.showErrorMessage('Package description is required.');
        return;
    }

    // Package type to create the folder structure
    const selectedType = await vscode.window.showQuickPick(
        ['C++', 'Python', 'Interface'],
        { placeHolder: 'Select ROS2 package type' }
    );

    if (!selectedType) {
        vscode.window.showErrorMessage('Please select a ROS2 package type.');
        return;
    }

    // Maintainer's name
    const maintainerName = await vscode.window.showInputBox({
        prompt: 'Enter the maintainer\'s first and lastname.'
    });

    if (!maintainerName) {
        vscode.window.showErrorMessage('Maintainer name is required.');
        return;
    }

    // Maintainer's E-Mail
    const maintainerEmail = await vscode.window.showInputBox({
        prompt: 'Enter the maintainer\'s E-Mail address.'
    });

    if (!maintainerEmail) {
        vscode.window.showErrorMessage('Maintainer E-Mail is required.');
        return;
    }

    // Package license
    const selectedLicense = await vscode.window.showQuickPick(
        ["MIT",
         "BSD",
         "ISC",
         "Apache License 2.0",
         "GNU General Public License v3",
         "Not open source"],
        { placeHolder: 'Select license' }
    );

    if (!selectedLicense) {
        vscode.window.showErrorMessage('License is required.');
        return;
    }

    // Directory where packages are created
    // This assumes the user is in the 'colcon_ws' folder
	const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
        vscode.window.showErrorMessage('No workspace folder found.');
        return;
    }
	const packagePath = path.join(workspaceFolder.uri.fsPath, 'src', packageSlug);

    try {
        // Create the main package directory
        fs.mkdirSync(packagePath);

        if (selectedType === 'C++') {
            createCPPDirectories(packageSlug, packagePath);
            createCPPFiles(packageSlug, packagePath, packageDescription,
                maintainerEmail, maintainerName, selectedLicense);
        } else if (selectedType === 'Python') {
            createPythonDirectories(packageSlug, packagePath);
            createPythonFiles(packageSlug, packagePath, packageDescription,
                maintainerEmail, maintainerName, selectedLicense);
        } else if (selectedType === 'Interface') {
            createInterfaceDirectiores(packagePath);
            createInterfaceFiles(packageSlug, packagePath, packageDescription,
                maintainerEmail, maintainerName, selectedLicense);
        }

        vscode.window.showInformationMessage(`ROS2 package '${packageName}' created successfully!`);
    } catch (err) {
        vscode.window.showErrorMessage(`Error creating package: ${err}`);
    }
}

/**
* Create the directories of a ROS2 C++ package
* @param packageSlug slug of the package
* @param packagePath path of the package
*/
function createCPPDirectories(packageSlug: string, packagePath: string) {
    const directories = ['config',
                         'include',
                         'launch',
                         'resource',
                         'src',
                         path.join('include', packageSlug)];

    directories.forEach(dir => {
        fs.mkdirSync(path.join(packagePath, dir));
    });
}

/**
* Create the directories of a ROS2 Python package
* @param packageSlug slug of the package
* @param packagePath path of the package
*/
function createPythonDirectories(packageSlug: string, packagePath: string) {
    const directories = ['config', 'launch', 'resource', packageSlug];

    directories.forEach(dir => {
        fs.mkdirSync(path.join(packagePath, dir));
    });
}

/**
* Create the directories of a ROS2 Interface package
* @param packagePath path of the package
*/
function createInterfaceDirectiores(packagePath: string) {
    const directories = ['msg', 'srv', 'action', 'resource'];

    directories.forEach(dir => {
        fs.mkdirSync(path.join(packagePath, dir));
    });
}

// Content of the params.yaml file
const configFileContent = `{{PACKAGE_SLUG}}_node:
  ros__parameters:
    foo: 'This is the {{PACKAGE_SLUG}} package'`

// Content of the package.xml file
const xmlFileContent = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{{PACKAGE_SLUG}}</name>
  <version>0.1.0</version>
  <description>{{PACKAGE_DESCRIPTION}}</description>
  <maintainer email="{{MAINTAINER_EMAIL}}">{{MAINTAINER_NAME}}</maintainer>
  <license>{{LICENSE}}</license>

  <export>
    <build_type>{{AMENT_TYPE}}</build_type>
  </export>
</package>`

// Content of the CMakeLists.txt for C++ package
const cmakeListsFileContent = `cmake_minimum_required(VERSION 3.5)
project({{PACKAGE_SLUG}})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/\${PROJECT_NAME}
)

add_executable({{PACKAGE_SLUG}}_node src/{{PACKAGE_SLUG}}_node.cpp)
ament_target_dependencies({{PACKAGE_SLUG}}_node rclcpp)

install(TARGETS
  {{PACKAGE_SLUG}}_node
  DESTINATION lib/\${PROJECT_NAME})

ament_package()`

// Content of the node file for C++ package
const cppNodeFileContent = `#include "rclcpp/rclcpp.hpp"
#include <string>

class {{CLASS_NAME}} : public rclcpp::Node {
public:
    {{CLASS_NAME}} () : Node("{{PACKAGE_SLUG}}_node") {
        // Declare a parameter named "foo"
        this->declare_parameter<std::string>("foo", "bar");

        // Get the value of the parameter "foo"
        std::string foo_value;
        if (this->get_parameter("foo", foo_value)) {
            RCLCPP_INFO(this->get_logger(), "Value of 'foo' parameter: %s", foo_value);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get the 'foo' parameter.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<{{CLASS_NAME}}>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}`

// Content of the package.xml file for interface messages
const interfaceXmlFileContent = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{{PACKAGE_SLUG}}</name>
  <version>0.1.0</version>
  <description>{{PACKAGE_DESCRIPTION}}</description>
  <maintainer email="{{MAINTAINER_EMAIL}}">{{MAINTAINER_NAME}}</maintainer>
  <license>{{LICENSE}}</license>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>{{AMENT_TYPE}}</build_type>
  </export>
</package>`

// Content of the interface package CMakeLists.txt
const interfaceCmakeListsFileContent = `cmake_minimum_required(VERSION 3.5)
project({{PACKAGE_SLUG}})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Message1.msg"
)

set(srv_files
  "srv/Service1.srv"
)

set(action_files
  "action/Action1.action"
)

rosidl_generate_interfaces(\${PROJECT_NAME}
  \${msg_files}
  \${srv_files}
  \${action_files}
)

ament_package()`

// Content of the message file
const msgFileContent = `string example_string
int64 example_number`

// Content of the service file
const srvFileContent = `string cmd
---
bool success`

// Content of the action file
const actionFileContent = `string cmd
---
uint64 feedback
---
bool success
uint64 result`

// Content of the launch file
const launchFileContent = `from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

package_path = get_package_share_directory('{{PACKAGE_SLUG}}')

def generate_launch_description():
    ld = LaunchDescription()

    {{PACKAGE_SLUG}}_node = Node(
        package='{{PACKAGE_SLUG}}',
        executable='{{PACKAGE_SLUG}}_node',
        name='{{PACKAGE_SLUG}}_node',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters = [
                ParameterFile(
                    param_file = f'{package_path}/config/params.yaml',
                    allow_substs = True)]
    )

    ld.add_action({{PACKAGE_SLUG}}_node)

    return ld`

// Content of setup.cfg file
const setupcfgFileContent = `[develop]
script_dir=$base/lib/{{PACKAGE_SLUG}}
[install]
install_scripts=$base/lib/{{PACKAGE_SLUG}}`

// Content of setup.py
const setuppyFileContent = `from glob import glob
import os

from setuptools import setup

package_name = '{{PACKAGE_SLUG}}'

setup(
    name=package_name,find_package(geometry_msgs)
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='{{MAINTAINER_NAME}}',
    {{MAINTAINER_EMAIL}}='{{MAINTAINER_EMAIL}}',
    description='{{PACKAGE_DESCRIPTION}}',
    license='{{LICENSE}}',
    entry_points={
        'console_scripts': [
            '{{PACKAGE_SLUG}}_node = {{PACKAGE_SLUG}}.{{PACKAGE_SLUG}}_node:main'
        ],
    },
)`

// Content of the python node
const pythonNodeFileContent = `import rclpy
from rclpy.node import Node


class {{CLASS_NAME}}(Node):
    def __init__(self) -> None:
        super().__init__('{{PACKAGE_SLUG}}_node')

        # Do something
        self.declare_parameter("foo", "bar")
        foo = self.get_parameter("foo").get_parameter_value().string_value
        self.get_logger().info(foo)


def main():
    rclpy.init()
    node = {{CLASS_NAME}}()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
`

/**
* Create the files of a ROS2 C++ package
* @param packageSlug slug of the package
* @param packagePath path of the package
* @param packageDescription description of the package
* @param maintainerEmail E-Mail of the maintainer
* @param maintainerName first and last name of the maintainer
* @param selectedLicense license of the package
*/
function createCPPFiles(packageSlug: string,
                        packagePath: string,
                        packageDescription: string,
                        maintainerEmail: string,
                        maintainerName: string,
                        selectedLicense: string)
{
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // params.yaml
    fs.writeFileSync(path.join(packagePath, 'config', 'params.yaml'),
        configFileContent.replaceAll('{{PACKAGE_SLUG}}', `${packageSlug}`));

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        xmlFileContent.replace(
            '{{PACKAGE_SLUG}}', `${packageSlug}`).replace(
            '{{PACKAGE_DESCRIPTION}}', `${packageDescription}`).replace(
            '{{MAINTAINER_EMAIL}}', `${maintainerEmail}`).replace(
            '{{MAINTAINER_NAME}}', `${maintainerName}`).replace(
            '{{LICENSE}}', `${selectedLicense}`).replace(
            '{{AMENT_TYPE}}', 'ament_cmake')
    );

    // launch.py
    fs.writeFileSync(path.join(packagePath, 'launch', `${packageSlug}.launch.py`),
        launchFileContent.replaceAll('{{PACKAGE_SLUG}}', `${packageSlug}`));

    // CMakeLists.txt
    fs.writeFileSync(path.join(packagePath, 'CMakeLists.txt'),
        cmakeListsFileContent.replaceAll('{{PACKAGE_SLUG}}', packageSlug));

    // first node
    fs.writeFileSync(path.join(packagePath, 'src', `${packageSlug}_node.cpp`),
        cppNodeFileContent.replaceAll(
            '{{PACKAGE_SLUG}}', packageSlug).replaceAll(
            '{{CLASS_NAME}}', snakeToCamelCase(packageSlug)));
}

/**
* Create the files of a ROS2 Python package
* @param packageSlug slug of the package
* @param packagePath path of the package
* @param packageDescription description of the package
* @param maintainerEmail E-Mail of the maintainer
* @param maintainerName first and last name of the maintainer
* @param selectedLicense license of the package
*/
function createPythonFiles(packageSlug: string,
                           packagePath: string,
                           packageDescription: string,
                           maintainerEmail: string,
                           maintainerName: string,
                           selectedLicense: string)
{
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // __init__.py
    fs.writeFileSync(path.join(packagePath, packageSlug, '__init__.py'), '');

    // params.yaml
    fs.writeFileSync(path.join(packagePath, 'config', 'params.yaml'),
        configFileContent.replaceAll('{{PACKAGE_SLUG}}', packageSlug));

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        xmlFileContent.replace(
            '{{PACKAGE_SLUG}}', `${packageSlug}`).replace(
            '{{PACKAGE_DESCRIPTION}}', `${packageDescription}`).replace(
            '{{MAINTAINER_EMAIL}}', `${maintainerEmail}`).replace(
            '{{MAINTAINER_NAME}}', `${maintainerName}`).replace(
            '{{LICENSE}}', `${selectedLicense}`).replace(
            '{{AMENT_TYPE}}', 'ament_python')
    );

    // launch.py
    fs.writeFileSync(path.join(packagePath, 'launch', `${packageSlug}.launch.py`),
        launchFileContent.replaceAll('{{PACKAGE_SLUG}}', packageSlug));

    // setup.cfg
    fs.writeFileSync(path.join(packagePath, 'setup.cfg'),
        setupcfgFileContent.replaceAll('{{PACKAGE_SLUG}}', packageSlug));

    // setup.py
    fs.writeFileSync(path.join(packagePath, 'setup.py'),
        setuppyFileContent.replaceAll(
            '{{PACKAGE_SLUG}}', packageSlug).replace(
            '{{MAINTAINER_NAME}}', maintainerName).replace(
            '{{MAINTAINER_EMAIL}}', maintainerEmail).replace(
            '{{LICENSE}}', selectedLicense)
    );

    // first node
    fs.writeFileSync(path.join(packagePath, packageSlug, `${packageSlug}_node.py`),
        pythonNodeFileContent.replaceAll(
            '{{PACKAGE_SLUG}}', packageSlug).replaceAll(
            '{{CLASS_NAME}}', snakeToCamelCase(packageSlug)));
}

/**
* Create the files of a ROS2 Interface package
* @param packageSlug slug of the package
* @param packagePath path of the package
* @param packageDescription description of the package
* @param maintainerEmail E-Mail of the maintainer
* @param maintainerName first and last name of the maintainer
* @param selectedLicense license of the package
*/
function createInterfaceFiles(packageSlug: string,
                              packagePath: string,
                              packageDescription: string,
                              maintainerEmail: string,
                              maintainerName: string,
                              selectedLicense: string)
{
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        interfaceXmlFileContent.replace(
            '{{PACKAGE_SLUG}}', `${packageSlug}`).replace(
            '{{PACKAGE_DESCRIPTION}}', `${packageDescription}`).replace(
            '{{MAINTAINER_EMAIL}}', `${maintainerEmail}`).replace(
            '{{MAINTAINER_NAME}}', `${maintainerName}`).replace(
            '{{LICENSE}}', `${selectedLicense}`).replace(
            '{{AMENT_TYPE}}', 'ament_cmake')
    );

    // CMakeLists.txt
    fs.writeFileSync(path.join(packagePath, 'CMakeLists.txt'),
        interfaceCmakeListsFileContent.replace('{{PACKAGE_SLUG}}', packageSlug));

    // Message
    fs.writeFileSync(path.join(packagePath, 'msg', 'Message1.msg'), msgFileContent);

    // Service
    fs.writeFileSync(path.join(packagePath, 'srv', 'Service1.srv'), srvFileContent);

    // Action
    fs.writeFileSync(path.join(packagePath, 'action', 'Action1.action'), actionFileContent);
}

/**
 * Convert snake case tring to camel case string
 * @param input snake case string
 * @returns camel case string
 */
function snakeToCamelCase(input: string): string {
    return input.replace(/_([a-z])/g, function(match) {
        return match[1].toUpperCase();
    });
}