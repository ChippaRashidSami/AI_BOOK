# Data Contracts: Docusaurus UI Upgrade

## Content Data Contract

### Markdown Frontmatter Schema

All content pages in the `docs/` directory should follow this schema for frontmatter:

```yaml
title: string          # Required: Title of the document
sidebar_label?: string # Optional: Alternative text for sidebar navigation
description?: string   # Optional: SEO description of the content
keywords?: string[]    # Optional: SEO keywords
tags?: string[]        # Optional: Content tags for organization
sidebar_position?: number # Optional: Position in sidebar (lower numbers appear first)
```

### Content Page Schema

The processed content page structure will have:

```typescript
interface ContentPage {
  id: string;           // Unique identifier
  title: string;        // Title of the page
  content: string;      // HTML-converted content
  description?: string; // SEO description
  sidebar_label?: string; // Navigation label
  slug: string;         // URL slug
  permalink: string;    // Full URL path
  sidebar: string;      // Sidebar configuration reference
}
```

## Navigation Data Contract

### Sidebar Configuration Schema

The `sidebars.js` file should follow this structure:

```javascript
module.exports = {
  docs: {
    [category: string]: [
      {
        type: 'category' | 'doc' | 'link';
        label?: string;           // Display name
        id?: string;              // Document ID (for 'doc' type)
        href?: string;            // External URL (for 'link' type)
        items?: SidebarItem[];    // Nested items (for 'category' type)
        collapsed?: boolean;      // Whether the category is collapsed by default
      }
    ]
  }
};
```

## Component Props Contract

### Custom Components

If custom components are developed, they should follow these prop contracts:

#### Navigation Component
```typescript
interface NavigationProps {
  items: NavigationItem[];
  activePath?: string;
  className?: string;
}
```

#### Content Display Component
```typescript
interface ContentDisplayProps {
  content: string;
  className?: string;
  toc?: boolean;  // Whether to show table of contents
}
```

## Build Output Contract

The Docusaurus build process will produce:

```typescript
interface BuildOutput {
  status: 'success' | 'error';
  outputDir: string;        // Directory where static files are generated
  files: string[];          // List of generated files
  size: number;             // Total size in bytes
  performanceMetrics: {
    loadTime: number;       // Estimated load time in ms
    bundleSize: number;     // JavaScript bundle size in bytes
  };
}
```

## Plugin Interface Contract

Any custom plugins should implement:

```typescript
interface DocusaurusPlugin {
  name: string;
  loadContent?: () => Promise<any>;
  contentLoaded?: (content: any, actions: any) => void;
  configureWebpack?: (config, isServer, utils) => object;
  getThemePath?: () => string;
  getTypeScriptThemePath?: () => string;
  getPathsToWatch?: () => string[];
}
```