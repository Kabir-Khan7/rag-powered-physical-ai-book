import React from 'react';
import DocItemFooter from '@theme-original/DocItem/Footer';
import type {Props} from '@theme/DocItem/Footer';
import PersonalizeButton from '../../../components/PersonalizeButton';
import TranslateToggle from '../../../components/TranslateToggle';

export default function FooterWrapper(props: Props) {
  return (
    <>
      <TranslateToggle />
      <PersonalizeButton />
      <DocItemFooter {...props} />
    </>
  );
}
