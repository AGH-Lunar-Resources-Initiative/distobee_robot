import styles from './anime.module.css';

import distobeeAnime from '!!url-loader!../media/distobee-anime.jpg';

export default function Anime() {
  return (
    <div className={styles['anime-panel']}>
      <img
        src={distobeeAnime}
        draggable='false'
        alt=''
      />
    </div>
  );
}
