const { defineConfig } = require('cypress')

module.exports = defineConfig({
  component: {
    enable: true,
    devServer: {
      framework: 'react',
      bundler: 'vite'
    }
  },
  e2e: {
    baseUrl: 'http://localhost:3000',
    setupNodeEvents(on, config) {
      // implement node event listeners here
    }
  }
})