import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Complete Textbook',
    description: (
      <p>
        A comprehensive academic textbook covering Physical AI from foundations to advanced humanoid robotics systems.
      </p>
    ),
  },
  {
    title: 'Hands-on Learning',
    description: (
      <p>
        Practical exercises and reproducible examples with ROS 2, Gazebo, and NVIDIA Isaac Sim.
      </p>
    ),
  },
  {
    title: 'Cutting-edge Content',
    description: (
      <p>
        Covers the latest developments in Vision-Language-Action systems and autonomous humanoid robots.
      </p>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}