# Data Model: Docusaurus UI Upgrade

## Content Pages

**Description**: Represents the book content in Markdown format that needs to be displayed in the new UI

**Attributes**:
- id: Unique identifier for the content page
- title: Title of the content page
- content: Markdown content of the page
- path: URL path for the content page
- metadata: Additional metadata (author, date, tags, etc.)
- parent: Reference to parent content in the hierarchy
- children: References to child content in the hierarchy

**Relationships**:
- One-to-many: A parent content page can have multiple child content pages
- Many-to-one: Child content pages belong to a single parent content page

**Validation Rules**:
- Title must not be empty
- Content must be valid Markdown format
- Path must be unique within the site
- Path must follow the specified URL structure

## Navigation Structure

**Description**: Represents the hierarchical organization of content that determines sidebar and navbar structure

**Attributes**:
- id: Unique identifier for the navigation item
- title: Title of the navigation item
- path: URL path associated with the navigation item
- level: Hierarchy level (0 for top-level, increasing for deeper levels)
- order: Display order within the same level
- parent: Reference to parent navigation item
- children: References to child navigation items
- visible: Boolean indicating if the item should be displayed in navigation

**Relationships**:
- One-to-many: A parent navigation item can have multiple child navigation items
- Many-to-one: Child navigation items belong to a single parent navigation item

**Validation Rules**:
- Title must not be empty
- Path must correspond to an existing content page
- No circular references in the hierarchy
- Order values must be unique within the same parent

## UI Components

**Description**: Represents Docusaurus-based components that will render the content and navigation

**Attributes**:
- id: Unique identifier for the UI component
- name: Name of the component
- type: Type of component (layout, navigation, content display, etc.)
- props: Properties that configure the component
- template: Template or JSX code for the component

**Relationships**:
- None (these are functional components, not data entities)

**Validation Rules**:
- Name must be unique
- Type must be a valid component type
- Props must match the expected configuration for the component type