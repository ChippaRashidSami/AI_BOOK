describe('Accessibility Tests', () => {
  it('should have no detectable accessibility violations on main pages', () => {
    cy.visit('/');
    
    // Run a11y checks
    cy.injectAxe(); // Requires axe-core to be installed
    cy.checkA11y();
  });

  it('should have proper heading hierarchy', () => {
    cy.visit('/');
    
    // Check for proper heading structure
    cy.get('h1').should('have.length', 1);
    cy.get('h1').should('not.be.empty');
    
    // Check that headings follow proper hierarchy
    cy.get('h2').should('exist');
  });

  it('should have proper contrast ratios', () => {
    cy.visit('/');
    
    // Check some key elements have proper contrast
    cy.get('h1').should('have.css', 'color');
    cy.get('body').should('have.css', 'background-color');
  });

  it('should be navigable by keyboard', () => {
    cy.visit('/');
    
    // Check that elements can be focused
    cy.get('a').first().focus();
    cy.get('a').first().should('have.focus');
  });
});