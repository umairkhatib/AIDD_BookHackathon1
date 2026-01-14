#!/bin/bash

# deploy-website.sh - Deployment script for the Physical AI & Humanoid Robotics Course website
# This script builds and deploys the Docusaurus-based documentation site to GitHub Pages

set -e  # Exit on any error

echo "==========================================="
echo "Physical AI & Humanoid Robotics Course"
echo "Website Deployment Script"
echo "==========================================="

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
if ! command_exists git; then
    echo "ERROR: git is not installed."
    exit 1
fi

if ! command_exists npm; then
    echo "ERROR: npm is not installed."
    exit 1
fi

# Check if we're in the right directory
if [ ! -d "website" ] || [ ! -f "website/package.json" ]; then
    echo "ERROR: This script must be run from the project root directory."
    echo "Expected to find 'website' directory with package.json file."
    exit 1
fi

# Get current branch
CURRENT_BRANCH=$(git branch --show-current)
echo "Current branch: $CURRENT_BRANCH"

# Confirm deployment
echo ""
echo "This script will:"
echo "1. Build the Docusaurus website"
echo "2. Deploy to GitHub Pages"
echo ""
read -p "Do you want to continue? (y/N): " -n 1 -r REPLY
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Deployment cancelled."
    exit 0
fi

# Navigate to website directory
cd website

# Install dependencies
echo "Installing dependencies..."
npm ci

# Build the website
echo "Building the website..."
npm run build

# Check if build succeeded
if [ $? -ne 0 ]; then
    echo "Build failed. Aborting deployment."
    exit 1
fi

echo "Build completed successfully!"

# Check if we should deploy to GitHub Pages
read -p "Deploy to GitHub Pages? (y/N): " -n 1 -r DEPLOY_REPLY
echo ""

if [[ $DEPLOY_REPLY =~ ^[Yy]$ ]]; then
    # Configure git for deployment
    if [ -z "$GIT_USER_NAME" ] || [ -z "$GIT_USER_EMAIL" ]; then
        GIT_USER_NAME="${GIT_USER_NAME:-$(git config user.name)}"
        GIT_USER_EMAIL="${GIT_USER_EMAIL:-$(git config user.email)}"
    fi

    git config --global user.name "${GIT_USER_NAME:-'github-actions'}"
    git config --global user.email "${GIT_USER_EMAIL:-'github-actions@users.noreply.github.com'}"

    # Deploy to GitHub Pages
    echo "Deploying to GitHub Pages..."
    npx docusaurus deploy

    if [ $? -eq 0 ]; then
        echo "Deployment to GitHub Pages completed successfully!"
        echo "Your site should be available at: https://$(git config user.name).github.io/physical-ai-humanoid-course/"
    else
        echo "Deployment to GitHub Pages failed!"
        exit 1
    fi
else
    echo "Skipping GitHub Pages deployment."
fi

# Option to serve locally for review
read -p "Serve locally for review? (y/N): " -n 1 -r SERVE_REPLY
echo ""

if [[ $SERVE_REPLY =~ ^[Yy]$ ]]; then
    echo "Starting local server..."
    echo "Site will be available at http://localhost:3000"
    echo "Press Ctrl+C to stop the server."
    npm run serve
fi

echo ""
echo "Deployment script completed!"