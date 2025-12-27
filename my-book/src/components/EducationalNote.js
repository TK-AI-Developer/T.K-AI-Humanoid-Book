import React from 'react';
import styles from './EducationalNote.module.css';

// EducationalNote component for highlighting important concepts
const EducationalNote = ({ type = 'info', title, children }) => {
  const noteClasses = `${styles.note} ${styles[type]}`;
  const titleText = title || (type === 'warning' ? 'Warning' : type === 'danger' ? 'Important' : 'Note');

  return (
    <div className={noteClasses}>
      <div className={styles.header}>
        <span className={styles.title}>{titleText}</span>
      </div>
      <div className={styles.content}>
        {children}
      </div>
    </div>
  );
};

export default EducationalNote;