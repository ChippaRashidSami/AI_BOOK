describe('Navigation Functionality', () => {
  it('should navigate between pages using sidebar and navbar', () => {
    cy.visit('/');
    
    // Test navbar navigation
    cy.get('nav').should('be.visible');
    cy.get('nav a').should('have.length.at.least', 1);
    
    // Test sidebar navigation
    cy.get('.menu').should('be.visible');
    cy.get('.menu__list').should('exist');
    
    // Test that navigation links work
    cy.get('nav a').first().click();
    cy.url().should('not.include', '/');
  });

  it('should adapt navigation for mobile screen sizes', () => {
    cy.visit('/');
    cy.viewport('iphone-6');
    
    // Check if mobile navigation is present
    cy.get('.navbar__toggle').should('be.visible');
    
    // Toggle mobile menu and verify it works
    cy.get('.navbar__toggle').click();
    cy.get('.navbar-sidebar').should('be.visible');
  });
});