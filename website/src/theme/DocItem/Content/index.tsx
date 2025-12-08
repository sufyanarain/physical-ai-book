import React, { useState, useEffect } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import TranslateButton from '@site/src/components/TranslateButton';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const [originalContent, setOriginalContent] = useState<string>('');
  const [translatedContent, setTranslatedContent] = useState<string>('');
  const [isTranslated, setIsTranslated] = useState(false);

  useEffect(() => {
    // Extract the main content from the page
    const contentElement = document.querySelector('.markdown');
    if (contentElement) {
      setOriginalContent(contentElement.textContent || '');
    }
  }, []);

  const handleTranslate = (translated: string) => {
    if (translated) {
      setTranslatedContent(translated);
      setIsTranslated(true);

      // Replace content with translation
      const contentElement = document.querySelector('.markdown');
      if (contentElement) {
        // Store original HTML if not already stored
        if (!contentElement.getAttribute('data-original-html')) {
          contentElement.setAttribute('data-original-html', contentElement.innerHTML);
        }

        // Show translated content
        const translatedDiv = document.createElement('div');
        translatedDiv.className = 'translated-content';
        translatedDiv.style.direction = 'rtl'; // Right-to-left for Urdu
        translatedDiv.style.textAlign = 'right';
        translatedDiv.style.fontFamily = 'Noto Nastaliq Urdu, Arial, sans-serif';
        translatedDiv.innerHTML = translated.replace(/\n/g, '<br>');

        contentElement.innerHTML = '';
        contentElement.appendChild(translatedDiv);
      }
    } else {
      // Restore original content
      setIsTranslated(false);
      const contentElement = document.querySelector('.markdown');
      if (contentElement) {
        const originalHtml = contentElement.getAttribute('data-original-html');
        if (originalHtml) {
          contentElement.innerHTML = originalHtml;
        }
      }
    }
  };

  return (
    <>
      <div style={{
        borderBottom: '1px solid var(--ifm-color-emphasis-300)',
        paddingBottom: '1rem',
        marginBottom: '1.5rem'
      }}>
        <TranslateButton
          content={originalContent}
          onTranslate={handleTranslate}
        />
      </div>
      <Content {...props} />
    </>
  );
}
