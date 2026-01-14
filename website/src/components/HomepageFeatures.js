import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Humanoid Robotics',
    description: (
      <>
        Learn to build embodied AI systems that operate in the physical world,
        combining robotics, artificial intelligence, and human-robot interaction.
      </>
    ),
  },
  {
    title: 'Complete Course Curriculum',
    description: (
      <>
        From ROS 2 fundamentals to Vision-Language-Action systems, master all aspects
        of humanoid robotics development.
      </>
    ),
  },
  {
    title: 'Hands-on Learning',
    description: (
      <>
        Practical exercises with Isaac Sim, real robotics challenges, and
        interactive demonstrations throughout the course.
      </>
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