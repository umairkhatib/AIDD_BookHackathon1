import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

/**
 * Course Navigation and Progress Tracking Component
 * Provides students with an overview of course progress and navigation
 */
const CourseNavigation = () => {
  const { siteConfig } = useDocusaurusContext();
  const [progressData, setProgressData] = useState({});

  // Course modules structure
  const courseModules = [
    {
      id: 'module-1',
      title: 'ROS 2 Fundamentals',
      path: '/docs/module-1-ros-foundations/intro',
      lessons: [
        { id: 'm1-l1', title: 'Introduction to ROS 2', path: '/docs/module-1-ros-foundations/intro' },
        { id: 'm1-l2', title: 'Nodes and Topics', path: '/docs/module-1-ros-foundations/concepts/ros2-architecture' },
        { id: 'm1-l3', title: 'Services and Actions', path: '/docs/module-1-ros-foundations/concepts/ros2-services' },
        { id: 'm1-l4', title: 'Python Implementation', path: '/docs/module-1-ros-foundations/implementation/python-examples' },
        { id: 'm1-l5', title: 'Exercises', path: '/docs/module-1-ros-foundations/exercises/ros2-exercises' }
      ]
    },
    {
      id: 'module-2',
      title: 'Digital Twin Simulation',
      path: '/docs/module-2-digital-twin/intro',
      lessons: [
        { id: 'm2-l1', title: 'Simulation Concepts', path: '/docs/module-2-digital-twin/concepts/simulation-fundamentals' },
        { id: 'm2-l2', title: 'Gazebo Integration', path: '/docs/module-2-digital-twin/implementation/gazebo-integration' },
        { id: 'm2-l3', title: 'Isaac Sim Setup', path: '/docs/module-2-digital-twin/implementation/isaac-sim-setup' },
        { id: 'm2-l4', title: 'Physics Simulation', path: '/docs/module-2-digital-twin/concepts/physics-simulation' },
        { id: 'm2-l5', title: 'Exercises', path: '/docs/module-2-digital-twin/exercises/simulation-exercises' }
      ]
    },
    {
      id: 'module-3',
      title: 'AI Brain - Perception & Navigation',
      path: '/docs/module-3-ai-brain/intro',
      lessons: [
        { id: 'm3-l1', title: 'Perception Systems', path: '/docs/module-3-ai-brain/concepts/perception-systems' },
        { id: 'm3-l2', title: 'Isaac Tools Integration', path: '/docs/module-3-ai-brain/implementation/isaac-tools-examples' },
        { id: 'm3-l3', title: 'Navigation Systems', path: '/docs/module-3-ai-brain/concepts/navigation-systems' },
        { id: 'm3-l4', title: 'SLAM Implementation', path: '/docs/module-3-ai-brain/implementation/slam-implementation' },
        { id: 'm3-l5', title: 'Exercises', path: '/docs/module-3-ai-brain/exercises/navigation-exercises' }
      ]
    },
    {
      id: 'module-4',
      title: 'Vision-Language-Action Systems',
      path: '/docs/module-4-vla/intro',
      lessons: [
        { id: 'm4-l1', title: 'VLA Concepts', path: '/docs/module-4-vla/concepts/vla-fundamentals' },
        { id: 'm4-l2', title: 'Whisper Integration', path: '/docs/module-4-vla/implementation/whisper-integration' },
        { id: 'm4-l3', title: 'LLM Integration', path: '/docs/module-4-vla/implementation/llm-integration' },
        { id: 'm4-l4', title: 'VLA Integration', path: '/docs/module-4-vla/integration' },
        { id: 'm4-l5', title: 'Exercises', path: '/docs/module-4-vla/exercises/vla-exercises' }
      ]
    },
    {
      id: 'capstone',
      title: 'Capstone Project',
      path: '/docs/capstone-project/overview',
      lessons: [
        { id: 'c-l1', title: 'Overview', path: '/docs/capstone-project/overview' },
        { id: 'c-l2', title: 'Architecture', path: '/docs/capstone-project/architecture' },
        { id: 'c-l3', title: 'Implementation', path: '/docs/capstone-project/implementation' },
        { id: 'c-l4', title: 'Integration', path: '/docs/capstone-project/integration' },
        { id: 'c-l5', title: 'Assessment', path: '/docs/capstone-project/assessment' }
      ]
    }
  ];

  // Load progress from localStorage
  useEffect(() => {
    const savedProgress = localStorage.getItem('courseProgress');
    if (savedProgress) {
      setProgressData(JSON.parse(savedProgress));
    }
  }, []);

  // Save progress to localStorage
  const markLessonComplete = (lessonId) => {
    const newProgress = {
      ...progressData,
      [lessonId]: !progressData[lessonId] // Toggle completion status
    };
    setProgressData(newProgress);
    localStorage.setItem('courseProgress', JSON.stringify(newProgress));
  };

  // Calculate module progress
  const calculateModuleProgress = (module) => {
    const completedLessons = module.lessons.filter(lesson => progressData[lesson.id]).length;
    return Math.round((completedLessons / module.lessons.length) * 100);
  };

  // Calculate overall course progress
  const calculateOverallProgress = () => {
    const totalLessons = courseModules.reduce((total, module) => total + module.lessons.length, 0);
    const completedLessons = courseModules.reduce((completed, module) => {
      return completed + module.lessons.filter(lesson => progressData[lesson.id]).length;
    }, 0);
    return Math.round((completedLessons / totalLessons) * 100);
  };

  return (
    <div className="course-navigation-container" style={{ padding: '20px 0' }}>
      <div className="container">
        <h2>Course Navigation & Progress Tracking</h2>
        <p>Track your progress through the Physical AI & Humanoid Robotics course.</p>

        {/* Overall Progress */}
        <div className="row" style={{ marginBottom: '20px' }}>
          <div className="col">
            <div className="progress-card">
              <h3>Overall Course Progress</h3>
              <div className="progress-bar">
                <div
                  className="progress-fill"
                  style={{
                    width: `${calculateOverallProgress()}%`,
                    backgroundColor: '#4caf50',
                    height: '20px',
                    borderRadius: '10px',
                    transition: 'width 0.3s ease'
                  }}
                >
                  <span style={{ color: 'white', paddingLeft: '10px', fontSize: '12px' }}>
                    {calculateOverallProgress()}%
                  </span>
                </div>
              </div>
              <p>{calculateOverallProgress()}% Complete</p>
            </div>
          </div>
        </div>

        {/* Module List */}
        {courseModules.map((module, moduleIndex) => (
          <div key={module.id} className="module-card" style={{
            border: '1px solid #ddd',
            borderRadius: '8px',
            marginBottom: '20px',
            padding: '20px',
            backgroundColor: '#f9f9f9'
          }}>
            <div className="row">
              <div className="col col--8">
                <h3>
                  <Link to={module.path}>
                    {moduleIndex + 1}. {module.title}
                  </Link>
                </h3>
              </div>
              <div className="col col--4">
                <div className="module-progress">
                  <div className="progress-bar">
                    <div
                      className="progress-fill"
                      style={{
                        width: `${calculateModuleProgress(module)}%`,
                        backgroundColor: '#2196f3',
                        height: '16px',
                        borderRadius: '8px'
                      }}
                    >
                      <span style={{ color: 'white', paddingLeft: '5px', fontSize: '10px' }}>
                        {calculateModuleProgress(module)}%
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            {/* Lessons List */}
            <div className="lessons-list" style={{ marginTop: '15px' }}>
              <h4>Lessons:</h4>
              <ul style={{ listStyle: 'none', padding: 0 }}>
                {module.lessons.map((lesson, lessonIndex) => (
                  <li key={lesson.id} style={{
                    marginBottom: '8px',
                    padding: '8px',
                    backgroundColor: 'white',
                    borderRadius: '4px',
                    border: '1px solid #eee'
                  }}>
                    <div className="row">
                      <div className="col col--10">
                        <Link to={lesson.path}>
                          {lessonIndex + 1}. {lesson.title}
                        </Link>
                      </div>
                      <div className="col col--2">
                        <button
                          onClick={() => markLessonComplete(lesson.id)}
                          style={{
                            backgroundColor: progressData[lesson.id] ? '#4caf50' : '#f44336',
                            color: 'white',
                            border: 'none',
                            borderRadius: '4px',
                            padding: '4px 8px',
                            cursor: 'pointer',
                            fontSize: '12px'
                          }}
                        >
                          {progressData[lesson.id] ? 'Completed' : 'Mark Complete'}
                        </button>
                      </div>
                    </div>
                  </li>
                ))}
              </ul>
            </div>
          </div>
        ))}

        {/* Progress Summary */}
        <div className="progress-summary" style={{
          border: '1px solid #ddd',
          borderRadius: '8px',
          padding: '20px',
          backgroundColor: '#f0f8ff'
        }}>
          <h3>Progress Summary</h3>
          <div className="row">
            <div className="col col--4">
              <p><strong>Total Modules:</strong> {courseModules.length}</p>
            </div>
            <div className="col col--4">
              <p><strong>Total Lessons:</strong> {courseModules.reduce((total, module) => total + module.lessons.length, 0)}</p>
            </div>
            <div className="col col--4">
              <p><strong>Completed:</strong> {Object.keys(progressData).filter(id => progressData[id]).length} lessons</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default CourseNavigation;