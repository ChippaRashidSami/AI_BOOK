describe('URL Preservation Tests', () => {
  it('should maintain existing URLs for all content', () => {
    // Test that existing URLs still work after the Docusaurus upgrade
    cy.visit('/docs/intro');
    cy.url().should('include', '/docs/intro');
    
    // Test various curriculum module URLs
    cy.visit('/docs/ros2-robotics-module/index');
    cy.url().should('include', '/docs/ros2-robotics-module/index');
    
    cy.visit('/docs/ros2-robotics-module/ros2-foundations');
    cy.url().should('include', '/docs/ros2-robotics-module/ros2-foundations');
    
    cy.visit('/docs/digital-twin-sim/physics-simulation');
    cy.url().should('include', '/docs/digital-twin-sim/physics-simulation');
    
    cy.visit('/docs/ai-robot-brain/isaac-sim');
    cy.url().should('include', '/docs/ai-robot-brain/isaac-sim');
    
    cy.visit('/docs/vla-integration/voice-to-action');
    cy.url().should('include', '/docs/vla-integration/voice-to-action');
  });

  it('should handle deep links correctly', () => {
    // Test deep linking to specific sections
    cy.visit('/docs/intro#setup');
    cy.url().should('include', '/docs/intro#setup');
    
    cy.visit('/docs/ros2-robotics-module/ros2-foundations#basic-concepts');
    cy.url().should('include', '/docs/ros2-robotics-module/ros2-foundations#basic-concepts');
  });

  it('should redirect properly if URL structures changed', () => {
    // If any redirects were implemented, test them here
    // For now, just verify that main paths work
    cy.request('/docs/intro').its('status').should('eq', 200);
    cy.request('/docs/ros2-robotics-module/index').its('status').should('eq', 200);
  });
});