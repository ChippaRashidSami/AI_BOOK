import React from 'react';
import { render, screen } from '@testing-library/react';
import ResponsiveUI from './ResponsiveUI';

describe('ResponsiveUI Component', () => {
  test('renders responsive layout elements', () => {
    render(<ResponsiveUI />);
    
    // Check if responsive elements are present
    expect(screen.getByTestId('responsive-container')).toBeInTheDocument();
  });

  test('applies correct CSS classes for responsiveness', () => {
    render(<ResponsiveUI />);
    
    const container = screen.getByTestId('responsive-container');
    expect(container).toHaveClass('responsive-container');
    expect(container).toHaveClass('container-fluid'); // Common responsive class
  });
});