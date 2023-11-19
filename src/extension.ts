import * as vscode from 'vscode';
import * as fs from 'fs';
import * as path from 'path';

export function activate(context: vscode.ExtensionContext) {
    let disposable = vscode.commands.registerCommand('ros2-gde.create-package', async () => {
        const packageName = await vscode.window.showInputBox({
            prompt: 'Enter the name for your ROS2 package'
        });

        if (!packageName) {
            vscode.window.showErrorMessage('Package name is required.');
            return;
        }

		const selectedType = await vscode.window.showQuickPick(
            ['roscpp', 'rospy'],
            { placeHolder: 'Select ROS2 package type' }
        );

        if (!selectedType) {
            vscode.window.showErrorMessage('Please select a ROS2 package type.');
            return;
        }

        // Directory where packages are created
		const workspaceFolder = vscode.workspace.workspaceFolders?.[0];
        if (!workspaceFolder) {
            vscode.window.showErrorMessage('No workspace folder found.');
            return;
        }
		const packagePath = path.join(workspaceFolder.uri.fsPath, 'src', packageName);

        try {
            // Create the main package directory
            fs.mkdirSync(packagePath);

            // Create subdirectories (src, include, launch, etc.)
            createDirectories(packagePath);

            // Create necessary files (CMakeLists.txt, package.xml, etc.)
            createFiles(packagePath, packageName);

            vscode.window.showInformationMessage(`ROS2 package '${packageName}' created successfully!`);
        } catch (err) {
            vscode.window.showErrorMessage(`Error creating package: ${err}`);
        }
    });

    context.subscriptions.push(disposable);
}

function createDirectories(packagePath: string) {
    const directories = ['src', 'include', 'launch', 'msg', 'srv'];

    directories.forEach(dir => {
        fs.mkdirSync(path.join(packagePath, dir));
    });
}

function createFiles(packagePath: string, packageName: string) {
    const cmakeFileContent = `# CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(${packageName})

# ... (add your CMake configurations here)
`;

    const packageXmlContent = `<?xml version="1.0"?>
<package format="3">
  <name>${packageName}</name>
  <version>0.0.0</version>
  <description>ROS2 package</description>
  <!-- ... (add your package details here) -->
</package>`;

    fs.writeFileSync(path.join(packagePath, 'CMakeLists.txt'), cmakeFileContent);
    fs.writeFileSync(path.join(packagePath, 'package.xml'), packageXmlContent);
}