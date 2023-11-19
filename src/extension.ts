import * as vscode from 'vscode';
import { createPackage } from './commands/createPackage';

export function activate(context: vscode.ExtensionContext) {
    let disposable = vscode.commands.registerCommand('ros2-gde.create-package', createPackage)
	context.subscriptions.push(disposable);
}