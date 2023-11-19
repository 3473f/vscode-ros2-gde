import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export async function createPackage() {

    const packageName = await vscode.window.showInputBox({
        prompt: 'Enter the name for your ROS2 package'
    });

    if (!packageName) {
        vscode.window.showErrorMessage('Package name is required.');
        return;
    }

    const selectedType = await vscode.window.showQuickPick(
        ['C++', 'Python', 'Interface'],
        { placeHolder: 'Select ROS2 package type' }
    );

    if (!selectedType) {
        vscode.window.showErrorMessage('Please select a ROS2 package type.');
        return;
    }

    // Directory where packages are created
    // This assumes the user is in the 'colcon_ws' folder
	const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
    if (!workspaceFolder) {
        vscode.window.showErrorMessage('No workspace folder found.');
        return;
    }
	const packagePath = path.join(workspaceFolder.uri.fsPath, 'src', packageName);

    try {
        // Create the main package directory
        fs.mkdirSync(packagePath);

        if (selectedType === 'C++') {
            createCPPDirectories(packagePath, packageName);
        //    createCPPFiles(packagePath, packageName);
        } else if (selectedType === 'Python') {
            createPythonDirectories(packagePath, packageName);
        //    createPyFiles(packagePath, packageName);
        } else if (selectedType === 'Interface') {
            createInterfaceDirectiores(packagePath);
        }

        vscode.window.showInformationMessage(`ROS2 package '${packageName}' created successfully!`);
    } catch (err) {
        vscode.window.showErrorMessage(`Error creating package: ${err}`);
    }
}

/**
* Create the directories of a ROS2 C++ package
* @param packageName name of the package
* @param packagePath path of the package
*/
function createCPPDirectories(packageName: string, packagePath: string) {
    const directories = ['include',
                         'launch',
                         'resource',
                         'src',
                         path.join('include', packageName)];

    directories.forEach(dir => {
        fs.mkdirSync(path.join(packagePath, dir));
    });
}

/**
* Create the directories of a ROS2 Python package
* @param packageName name of the package
* @param packagePath path of the package
*/
function createPythonDirectories(packageName: string, packagePath: string) {
    const directories = [packageName, 'launch', 'resource'];

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