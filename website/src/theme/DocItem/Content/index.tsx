import React, { useState, useEffect } from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import TranslateButton from '@site/src/components/TranslateButton';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import ReactMarkdown from 'react-markdown';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const [originalContent, setOriginalContent] = useState<string>('');
  const [translatedContent, setTranslatedContent] = useState<string>('');
  const [isTranslated, setIsTranslated] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);

  useEffect(() => {
    // Extract the main content from the page
    const contentElement = document.querySelector('.markdown');
    if (contentElement) {
      // Get text content for translation
      setOriginalContent(contentElement.textContent || '');
    }
  }, [props]);

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

        // Create translated content container
        const translatedDiv = document.createElement('div');
        translatedDiv.className = 'translated-content';
        
        // Parse markdown-like content for better formatting
        const formattedContent = translated
          .split('\n\n')
          .map(para => {
            // Preserve code blocks
            if (para.trim().startsWith('```') || para.includes('`')) {
              return `<pre style="direction: ltr; text-align: left;">${para}</pre>`;
            }
            return `<p>${para}</p>`;
          })
          .join('');

        translatedDiv.innerHTML = formattedContent;
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

  const handlePersonalize = (personalizedContent: string) => {
    if (personalizedContent) {
      setIsPersonalized(true);

      // Replace content with personalized version
      const contentElement = document.querySelector('.markdown');
      if (contentElement) {
        // Store original HTML if not already stored
        if (!contentElement.getAttribute('data-original-html')) {
          contentElement.setAttribute('data-original-html', contentElement.innerHTML);
        }

        // Create personalized content container
        const personalizedDiv = document.createElement('div');
        personalizedDiv.className = 'personalized-content';
        
        // Format the personalized content
        const formattedContent = personalizedContent
          .split('\n\n')
          .map(para => {
            // Preserve code blocks
            if (para.trim().startsWith('```')) {
              const codeMatch = para.match(/```(\w+)?\n([\s\S]*?)```/);
              if (codeMatch) {
                return `<pre><code class="language-${codeMatch[1] || 'text'}">${codeMatch[2]}</code></pre>`;
              }
              return `<pre>${para.replace(/```/g, '')}</pre>`;
            }
            // Preserve inline code
            const withCode = para.replace(/`([^`]+)`/g, '<code>$1</code>');
            return `<p>${withCode}</p>`;
          })
          .join('');

        personalizedDiv.innerHTML = formattedContent;
        contentElement.innerHTML = '';
        contentElement.appendChild(personalizedDiv);
      }
    } else {
      // Restore original content
      setIsPersonalized(false);
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
      <div className="translate-button-wrapper">
        <PersonalizeButton
          content={originalContent}
          onPersonalize={handlePersonalize}
        />
        <TranslateButton
          content={originalContent}
          onTranslate={handleTranslate}
        />
      </div>
      <Content {...props} />
    </>
  );
}
