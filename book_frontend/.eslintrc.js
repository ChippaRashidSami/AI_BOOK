module.exports = {
  env: {
    browser: true,
    es2021: true,
    node: true,
  },
  extends: [
    '@eslint/js',
    'plugin:react/recommended',
    'plugin:react-hooks/recommended',
    'plugin:jsx-a11y/recommended',
  ],
  parserOptions: {
    ecmaFeatures: {
      jsx: true,
    },
    ecmaVersion: 12,
    sourceType: 'module',
  },
  plugins: ['react', 'jsx-a11y'],
  settings: {
    react: {
      version: 'detect',
    },
  },
  rules: {
    'react/react-in-jsx-scope': 'off', // Not needed in Docusaurus since React is auto-imported
    'react/prop-types': 'off', // Docusaurus handles this differently
  },
  overrides: [
    {
      files: ['*.js', '*.jsx'],
      excludedFiles: ['docusaurus.config.js'],
    },
  ],
};