import Anime from './anime';
import Feeds from './feeds';
import Pipes from './pipes';
import Sieve from './sieve';
import Wheels from './wheels';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { faGear, faVial, faVideo, faMicroscope, faHeart } from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'pipes' | 'wheels' | 'feeds' | 'science' | 'anime';
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
  },
  anime: {
    Component: Anime,
    name: 'For You <3',
    icon: faHeart
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
