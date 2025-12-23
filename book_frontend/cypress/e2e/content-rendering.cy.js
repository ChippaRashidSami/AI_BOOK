describe('Content Rendering Tests', () => {
  it('should render all existing content correctly', () => {
    // Visit the main docs page
    cy.visit('/docs/intro');
    
    // Check that content is present and properly formatted
    cy.get('.markdown').should('be.visible');
    cy.get('.markdown h1').should('exist');
    cy.get('.markdown p').should('have.length.greaterThan', 0);
    
    // Verify images render properly
    cy.get('img').should('have.attr', 'src').and('not.be.empty');
    
    // Verify code blocks are properly formatted
    cy.get('code').should('exist');
    cy.get('pre').should('exist');
  });

  it('should render content from all curriculum modules', () => {
    // Test Module 1 content
    cy.visit('/docs/ros2-robotics-module/index');
    cy.get('.markdown h1').should('be.visible');
    
    // Test Module 2 content
    cy.visit('/docs/digital-twin-sim/index');
    cy.get('.markdown h1').should('be.visible');
    
    // Test Module 3 content
    cy.visit('/docs/ai-robot-brain/index');
    cy.get('.markdown h1').should('be.visible');
    
    // Test Module 4 content
    cy.visit('/docs/vla-integration/index');
    cy.get('.markdown h1').should('be.visible');
  });
});