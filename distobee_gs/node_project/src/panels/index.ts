import Feeds from './feeds';
import Science from './science';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faVideo,
  faMicroscope
} from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'feeds' | 'science';
export const defaultPanel: PanelID = 'science';
export const panelInfos: Panels = {
  feeds: {
    Component: Feeds,
    name: 'Feeds',
    icon: faVideo
  },
  science: {
    Component: Science,
    name: 'Science',
    icon: faMicroscope
  }
};

// type definitions for the panels above
export type Panels = {
  [id in PanelID]: PanelInfo;
};
export type PanelInfo = {
  Component: React.ComponentType<any>;
  HeaderComponent?: React.ComponentType<any>;
  name: string;
  icon: IconDefinition;
};
