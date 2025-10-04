import Feeds from './feeds';
import Sieve from './sieve';
import Wheels from "./wheels";
import Pipes from "./pipes";
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faGear,
  faVial,
  faVideo,
  faMicroscope
} from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'pipes' | 'wheels' | 'feeds' | 'science';
export const defaultPanel: PanelID = 'science';
export const panelInfos: Panels = {
  pipes: {
    Component: Pipes,
    name: 'Pipes',
    icon: faVial
  },
  wheels: {
    Component: Wheels,
    name: 'Wheels',
    icon: faGear
  },
  feeds: {
    Component: Feeds,
    name: 'Feeds',
    icon: faVideo
  },
  science: {
    Component: Sieve,
    name: 'Sieve',
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
