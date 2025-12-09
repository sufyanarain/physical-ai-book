import React from 'react';
import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';
import PersonalizeButton from '../../../components/PersonalizeButton';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): React.ReactElement {
  return (
    <>
      <PersonalizeButton />
      <Content {...props} />
    </>
  );
}
