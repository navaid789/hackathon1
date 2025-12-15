import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.logoContainer}>
          <img
            src="/img/logo.svg"
            alt="Physical AI & Humanoid Robotics Logo"
            className={styles.logo}
          />
        </div>
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/chapter-1-introduction-to-physical-ai/">
            Read the Textbook - Chapter 1
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/chapter-1-introduction-to-physical-ai/">
            Start Learning
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI Textbook`}
      description="Academic textbook on Physical AI, Robotics, and Vision-Language-Action systems">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>Foundation</h2>
                <p>Learn the fundamentals of Physical AI, from digital AI to embodied intelligence and physics constraints.</p>
              </div>
              <div className="col col--4">
                <h2>Simulation & Middleware</h2>
                <p>Explore digital twins, physics simulation, and ROS 2 for robotic systems.</p>
              </div>
              <div className="col col--4">
                <h2>Advanced Systems</h2>
                <p>Dive into perception, navigation, reinforcement learning, and humanoid robotics.</p>
              </div>
            </div>
          </div>
        </section>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}