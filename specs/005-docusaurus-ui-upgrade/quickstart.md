# Quickstart Guide: Docusaurus UI Upgrade

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd book_frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   # or
   yarn install
   ```

3. Start the development server:
   ```bash
   npm run start
   # or
   yarn start
   ```

4. Open your browser to http://localhost:3000 to view the site

## Configuration

1. The main configuration is in `docusaurus.config.js`:
   - Site metadata (title, tagline, URL)
   - Theme configuration
   - Plugin configuration
   - Navigation configuration

2. Sidebar configuration is in `sidebars.js`:
   - Defines the navigation structure
   - Maps content to navigation items

## Development

1. To create a new content page:
   - Add a new `.md` file in the `docs/` directory
   - Follow the frontmatter format if needed:
     ```markdown
     ---
     title: Page Title
     sidebar_label: Sidebar Text
     ---
     ```

2. To run tests:
   ```bash
   npm run test
   # or
   yarn test
   ```

3. To build for production:
   ```bash
   npm run build
   # or
   yarn build
   ```

4. To serve the built site locally:
   ```bash
   npm run serve
   # or
   yarn serve
   ```

## Deployment

The site is configured for GitHub Pages deployment:

1. Build the site:
   ```bash
   npm run build
   ```

2. The output will be in the `build/` directory, ready for deployment to GitHub Pages