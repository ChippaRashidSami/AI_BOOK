import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import { useDoc } from '@docusaurus/theme-common/internal';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { translate } from '@docusaurus/Translate';

import './Navigation.css';

function Navigation() {
  const location = useLocation();
  const currentDoc = useDoc();
  
  // Define navigation structure
  const navigationItems = [
    { 
      label: 'Home', 
      to: '/', 
      isActive: location.pathname === '/' 
    },
    { 
      label: 'Curriculum', 
      to: '/docs/intro', 
      isActive: location.pathname.startsWith('/docs') 
    },
    { 
      label: 'Blog', 
      to: '/blog', 
      isActive: location.pathname.startsWith('/blog') 
    },
  ];

  return (
    <nav className={clsx(ThemeClassNames.docs.docMarkdown, 'navigation')}>
      <div className="navigation-container">
        <div className="navigation-items">
          {navigationItems.map((item, index) => (
            <Link
              key={index}
              to={item.to}
              className={clsx('navigation-item', {
                'navigation-item--active': item.isActive,
              })}
              aria-current={item.isActive ? 'page' : undefined}
            >
              {item.label}
            </Link>
          ))}
        </div>
      </div>
    </nav>
  );
}

export default Navigation;