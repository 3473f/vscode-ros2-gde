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
const configFileContent = `package_slug_node:
  ros__parameters:
    foo: 'This is the package_slug package'`

// Content of the package.xml file
const xmlFileContent = `<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>package_slug</name>
  <version>0.1.0</version>
  <description>package_description</description>
  <maintainer email="maintainer_email">maintainer_name</maintainer>
  <license>package_license</license>

  <export>
    <build_type>ament_type</build_type>
  </export>
</package>
`

// Content of the launch file
const launchFileContent = `from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

package_path = get_package_share_directory('package_slug')

def generate_launch_description():
    ld = LaunchDescription()

    package_slug_node = Node(
        package='package_slug',
        executable='package_slug_node',
        name='package_slug_node',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters = [
                ParameterFile(
                    param_file = f'{package_path}/config/params.yaml',
                    allow_substs = True)]
    )

    ld.add_action(package_slug_node)

    return ld
`
// Content of setup.cfg file
const setupcfgFileContent = `[develop]
script_dir=$base/lib/package_slug
[install]
install_scripts=$base/lib/package_slug
`

// Content of setup.py
const setuppyFileContent = `from glob import glob
import os

from setuptools import setup

package_name = 'package_slug'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='name_maintainer',
    maintainer_email='email_maintainer',
    description='package_description',
    license='selected_license',
    entry_points={
        'console_scripts': [
            'package_slug_node = package_slug.package_slug_node:main'
        ],
    },
)
`

// Content of the python node
const pythonNodeFileContent = `import rclpy
from rclpy.node import Node


class package_slug(Node):
    def __init__(self) -> None:
        super().__init__('package_slug_node')

        # Do something
        self.declare_parameter("foo", "bar")
        foo = self.get_parameter("foo").get_parameter_value().string_value
        self.get_logger().info(foo)


def main():
    rclpy.init()
    node = package_slug()
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
function createCPPFiles(packageSlug: string, packagePath: string, packageDescription: string,
    maintainerEmail: string, maintainerName: string, selectedLicense: string) {
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // params.yaml
    fs.writeFileSync(path.join(packagePath, 'config', 'params.yaml'),
        configFileContent.replaceAll('package_slug', `${packageSlug}`));

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        xmlFileContent.replace(
            'package_slug', `${packageSlug}`).replace(
            'package_description', `${packageDescription}`).replace(
            'maintainer_email', `${maintainerEmail}`).replace(
            'maintainer_name', `${maintainerName}`).replace(
            'package_license', `${selectedLicense}`).replace(
            'ament_type', 'ament_cmake')
    );

    // launch.py
    fs.writeFileSync(path.join(packagePath, 'launch', `${packageSlug}.launch.py`),
        launchFileContent.replaceAll('package_slug', `${packageSlug}`));

    // CMakeLists.txt
    fs.writeFileSync(path.join(packagePath, 'CMakeLists.txt'), '');

    // first node
    fs.writeFileSync(path.join(packagePath, 'src', `${packageSlug}_node.cpp`), '');
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
function createPythonFiles(packageSlug: string, packagePath: string, packageDescription: string,
    maintainerEmail: string, maintainerName: string, selectedLicense: string) {
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // __init__.py
    fs.writeFileSync(path.join(packagePath, packageSlug, '__init__.py'), '');

    // params.yaml
    fs.writeFileSync(path.join(packagePath, 'config', 'params.yaml'),
        configFileContent.replaceAll('package_slug', packageSlug));

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        xmlFileContent.replace(
            'package_slug', `${packageSlug}`).replace(
            'package_description', `${packageDescription}`).replace(
            'maintainer_email', `${maintainerEmail}`).replace(
            'maintainer_name', `${maintainerName}`).replace(
            'package_license', `${selectedLicense}`).replace(
            'ament_type', 'ament_python')
    );

    // launch.py
    fs.writeFileSync(path.join(packagePath, 'launch', `${packageSlug}.launch.py`),
        launchFileContent.replaceAll('package_slug', packageSlug));

    // setup.cfg
    fs.writeFileSync(path.join(packagePath, 'setup.cfg'),
        setupcfgFileContent.replaceAll('package_slug', packageSlug));

    // setup.py
    fs.writeFileSync(path.join(packagePath, 'setup.py'),
        setuppyFileContent.replaceAll(
            'package_slug', packageSlug).replace(
            'name_maintainer', maintainerName).replace(
            'email_maintainer', maintainerEmail).replace(
            'selected_license', selectedLicense)
    );

    // first node
    fs.writeFileSync(path.join(packagePath, packageSlug, `${packageSlug}_node.py`),
    pythonNodeFileContent.replaceAll('package_slug', packageSlug));
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
function createInterfaceFiles(packageSlug: string, packagePath: string, packageDescription: string,
    maintainerEmail: string, maintainerName: string, selectedLicense: string) {
    // resource
    fs.writeFileSync(path.join(packagePath, 'resource', packageSlug), '');

    // package.xml
    fs.writeFileSync(path.join(packagePath, 'package.xml'),
        xmlFileContent.replace(
            'package_slug', `${packageSlug}`).replace(
            'package_description', `${packageDescription}`).replace(
            'maintainer_email', `${maintainerEmail}`).replace(
            'maintainer_name', `${maintainerName}`).replace(
            'package_license', `${selectedLicense}`).replace(
            'ament_type', 'ament_cmake')
    );
}