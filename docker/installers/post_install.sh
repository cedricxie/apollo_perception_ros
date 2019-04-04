#!/usr/bin/env bash

echo "Finalizing the installation..."

# Clean up
#apt-get clean && rm -rf /var/lib/apt/lists/*

# Rosdep @ user level
rosdep update

# Install VSCode
echo "Installing vscode..."
cd ~/
wget https://go.microsoft.com/fwlink/?LinkID=760868 -O vscode.deb
sudo dpkg -i vscode.deb
sudo apt-get -fy install
rm vscode.deb

echo "Installation finished successfully!"