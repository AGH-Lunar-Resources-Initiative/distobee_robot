import styles from './anime.module.css';

import animeBerry from '!!url-loader!../media/anime-berry.png';
import animeDark from '!!url-loader!../media/anime-dark.png';
import animeLight from '!!url-loader!../media/anime-light.png';
import animePink from '!!url-loader!../media/anime-pink.png';

const themeImages = {
  dark: animeDark,
  light: animeLight,
  berry: animeBerry,
  pink: animePink
};

export default function Anime() {
  const theme = JSON.parse(localStorage.getItem('theme')) || 'dark';
  const imageSrc = themeImages[theme] || animeDark;

  return (
    <div className={styles['anime-panel']}>
      <img src={imageSrc} draggable='false' alt='' />
    </div>
  );
}
