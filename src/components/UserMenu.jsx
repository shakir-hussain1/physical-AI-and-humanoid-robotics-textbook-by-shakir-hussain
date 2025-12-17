import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import styles from './UserMenu.module.css';

export default function UserMenu({ user }) {
  const { logout } = useAuth();
  const [showMenu, setShowMenu] = useState(false);

  return (
    <div className={styles.userMenuContainer}>
      <div className={styles.userInfo}>
        <div className={styles.avatar}>
          {user?.full_name?.charAt(0) || user?.username?.charAt(0) || 'U'}
        </div>
        <div className={styles.userDetails}>
          <p className={styles.name}>{user?.full_name || user?.username}</p>
          <p className={styles.email}>{user?.email}</p>
        </div>
      </div>

      <div className={styles.menuToggle}>
        <button onClick={() => setShowMenu(!showMenu)} className={styles.menuBtn}>
          ⋮
        </button>

        {showMenu && (
          <div className={styles.dropdown}>
            <div className={styles.profileInfo}>
              <p><strong>Software:</strong> {user?.software_background}</p>
              <p><strong>Hardware:</strong> {user?.hardware_background}</p>
              <p><strong>Learning Style:</strong> {user?.preferred_learning_style}</p>
              {user?.programming_languages?.length > 0 && (
                <p><strong>Languages:</strong> {user.programming_languages.join(', ')}</p>
              )}
              {user?.interest_areas?.length > 0 && (
                <p><strong>Interests:</strong> {user.interest_areas.join(', ')}</p>
              )}
            </div>
            <button onClick={logout} className={styles.logoutBtn}>
              Logout
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
