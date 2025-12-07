import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import RAGChatbot from '../components/RAGChatbot';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div style={{marginTop: '1rem', fontSize: '0.9rem', opacity: 0.9}}>
          <p>Created by <strong>Faiza Siddiqui</strong> | ID: <strong>265718</strong></p>
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.ReactElement {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Master Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>🤖 Module 1: ROS 2</h3>
                  <p>
                    Master the robotic nervous system. Learn nodes, topics, services, and build distributed robot systems.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>🎮 Module 2: Simulation</h3>
                  <p>
                    Build digital twins with Gazebo and Unity. Test safely before deploying to real hardware.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>⚡ Module 3: NVIDIA Isaac</h3>
                  <p>
                    Leverage GPU-accelerated AI for perception, navigation, and manipulation.
                  </p>
                </div>
              </div>
            </div>
            <div className="row" style={{marginTop: '2rem'}}>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>🗣️ Module 4: VLA</h3>
                  <p>
                    Integrate voice, vision, and language models for natural robot control.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>💬 AI Chatbot</h3>
                  <p>
                    Ask questions anytime! Our RAG chatbot answers based on the textbook content.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>🎯 Capstone Project</h3>
                  <p>
                    Build an autonomous humanoid that understands voice commands and completes tasks.
                  </p>
                </div>
              </div>
            </div>
          </div>
        </section>
        
        <section className={styles.ctaSection}>
          <div className="container text--center">
            <h2>Ready to Build the Future?</h2>
            <p>Join thousands of students mastering Physical AI and Humanoid Robotics</p>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Begin Your Journey →
            </Link>
          </div>
        </section>
      </main>
      
      {/* RAG Chatbot */}
      <RAGChatbot />
    </Layout>
  );
}
