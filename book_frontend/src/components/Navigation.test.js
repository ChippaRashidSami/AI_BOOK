import React from 'react';
import { render, screen } from '@testing-library/react';
import { BrowserRouter } from 'react-router-dom';
import Navigation from './Navigation';

describe('Navigation Component', () => {
  test('renders navigation elements', () => {
    render(
      <BrowserRouter>
        <Navigation />
      </BrowserRouter>
    );
    
    // Check if navigation elements are present
    expect(screen.getByRole('navigation')).toBeInTheDocument();
  });

  test('displays correct navigation links', () => {
    render(
      <BrowserRouter>
        <Navigation />
      </BrowserRouter>
    );
    
    // Test for common navigation elements
    expect(screen.getByText('Tutorial')).toBeInTheDocument();
    expect(screen.getByText('Blog')).toBeInTheDocument();
  });
});