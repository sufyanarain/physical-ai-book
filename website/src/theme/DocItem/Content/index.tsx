import Content from '@theme-original/DocItem/Content';
import type ContentType from '@theme/DocItem/Content';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return <Content {...props} />;
}
