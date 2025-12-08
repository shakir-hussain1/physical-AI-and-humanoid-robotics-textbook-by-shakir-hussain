import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

const modules = [
  {
    title: 'Module 1: ROS 2 Foundations',
    description: 'Master Robot Operating System 2 - the industry standard for robotics development.',
    link: '/docs/modules/module-1-ros2',
    weeks: 'Weeks 3-4',
  },
  {
    title: 'Module 2: Digital Twin',
    description: 'Create virtual replicas with Gazebo and Unity for safe robot development.',
    link: '/docs/modules/module-2-digital-twin',
    weeks: 'Weeks 5-6',
  },
  {
    title: 'Module 3: NVIDIA Isaac',
    description: 'Leverage GPU-accelerated simulation and perception with NVIDIA Isaac.',
    link: '/docs/modules/module-3-isaac',
    weeks: 'Weeks 7-10',
  },
  {
    title: 'Module 4: VLA Systems',
    description: 'Build Vision-Language-Action systems for intelligent human-robot interaction.',
    link: '/docs/modules/module-4-vla',
    weeks: 'Weeks 11-13',
  },
];

function ModuleCard({title, description, link, weeks}) {
  return (
    <div className={clsx('col col--6', styles.moduleCard)}>
      <div className="card">
        <div className="card__header">
          <h3>{title}</h3>
          <span className="badge badge--primary">{weeks}</span>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link className="button button--primary button--block" to={link}>
            View Module
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="A comprehensive textbook for building intelligent humanoid robots">
      <HomepageHeader />
      <main>
        <section className={styles.modules}>
          <div className="container">
            <h2 className="text--center margin-bottom--lg">Course Modules</h2>
            <div className="row">
              {modules.map((props, idx) => (
                <ModuleCard key={idx} {...props} />
              ))}
            </div>
          </div>
        </section>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h3>16 Chapters</h3>
                <p>Comprehensive coverage from basics to advanced topics</p>
              </div>
              <div className="col col--4">
                <h3>30+ Code Examples</h3>
                <p>Working, tested code ready to run</p>
              </div>
              <div className="col col--4">
                <h3>Hands-on Exercises</h3>
                <p>Practice at Basic, Intermediate, and Advanced levels</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
