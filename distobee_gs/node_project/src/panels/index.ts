import Anime from './anime';
import Feeds from './feeds';
import Pipes from './pipes';
import Sifter from './sifter';
import Wheels from './wheels';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { faGear, faVial, faVideo, faMicroscope, faHeart } from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'pipes' | 'wheels' | 'feeds' | 'sifter' | 'anime';
export const defaultPanel: PanelID = 'anime';
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
  sifter: {
    Component: Sifter,
    name: 'Sifter',
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
