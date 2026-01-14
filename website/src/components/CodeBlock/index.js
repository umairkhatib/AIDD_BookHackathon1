import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

// A custom CodeBlock component for the Physical AI & Humanoid Robotics Course
// This component allows for embedding and executing code examples directly in the documentation

const CodeBlock = ({ children, language = 'python', title, showLineNumbers = false }) => {
  const { siteConfig } = useDocusaurusContext();

  // Default styling for the code block
  const codeBlockStyle = {
    backgroundColor: '#2d2d2d',
    color: '#f8f8f2',
    fontFamily: 'Monaco, Consolas, "Ubuntu Mono", monospace',
    fontSize: '14px',
    lineHeight: '1.5',
    padding: '16px',
    borderRadius: '6px',
    overflow: 'auto',
    margin: '16px 0',
    position: 'relative'
  };

  const titleStyle = {
    backgroundColor: '#1e1e1e',
    color: '#f8f8f2',
    padding: '4px 12px',
    borderRadius: '4px 4px 0 0',
    fontSize: '12px',
    fontWeight: 'bold',
    margin: '0',
    position: 'absolute',
    top: '-20px',
    left: '0'
  };

  const copyButtonStyle = {
    position: 'absolute',
    top: '8px',
    right: '8px',
    backgroundColor: '#444',
    color: '#fff',
    border: 'none',
    borderRadius: '4px',
    padding: '4px 8px',
    fontSize: '12px',
    cursor: 'pointer'
  };

  const copyToClipboard = () => {
    navigator.clipboard.writeText(children.trim());
  };

  return (
    <div style={{ position: 'relative' }}>
      {title && <h4 style={titleStyle}>{title}</h4>}
      <pre style={codeBlockStyle}>
        <code className={language ? `language-${language}` : ''}>
          {children}
        </code>
      </pre>
      <button style={copyButtonStyle} onClick={copyToClipboard}>
        Copy
      </button>
    </div>
  );
};

// ROS 2 Node Component - A special component to show ROS 2 examples
const ROS2NodeComponent = ({ nodeName, nodeCode, description }) => {
  return (
    <div style={{
      border: '1px solid #ddd',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#f9f9f9'
    }}>
      <h3 style={{ color: '#2e8555', margin: '0 0 12px 0' }}>
        ROS 2 Node: {nodeName}
      </h3>
      <p>{description}</p>
      <CodeBlock language="python" title={`${nodeName}.py`}>
        {nodeCode}
      </CodeBlock>
      <div style={{
        marginTop: '12px',
        padding: '8px',
        backgroundColor: '#e8f4f8',
        borderRadius: '4px',
        fontSize: '14px'
      }}>
        <strong>Usage:</strong> Run this node with: <code>ros2 run package_name {nodeName}</code>
      </div>
    </div>
  );
};

// Simulation Component - For showing Gazebo/Unity examples
const SimulationComponent = ({ simName, simConfig, description }) => {
  return (
    <div style={{
      border: '1px solid #ddd',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#f0f8f0'
    }}>
      <h3 style={{ color: '#2e8555', margin: '0 0 12px 0' }}>
        Simulation: {simName}
      </h3>
      <p>{description}</p>
      <CodeBlock language="xml" title={`${simName}.world`}>
        {simConfig}
      </CodeBlock>
      <div style={{
        marginTop: '12px',
        padding: '8px',
        backgroundColor: '#e8f4f8',
        borderRadius: '4px',
        fontSize: '14px'
      }}>
        <strong>Usage:</strong> Launch this simulation with: <code>ign gazebo {simName}.world</code>
      </div>
    </div>
  );
};

// URDF Model Component - For showing robot models
const URDFModelComponent = ({ modelName, urdfCode, description }) => {
  return (
    <div style={{
      border: '1px solid #ddd',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#f8f0f0'
    }}>
      <h3 style={{ color: '#2e8555', margin: '0 0 12px 0' }}>
        URDF Model: {modelName}
      </h3>
      <p>{description}</p>
      <CodeBlock language="xml" title={`${modelName}.urdf`}>
        {urdfCode}
      </CodeBlock>
      <div style={{
        marginTop: '12px',
        padding: '8px',
        backgroundColor: '#e8f4f8',
        borderRadius: '4px',
        fontSize: '14px'
      }}>
        <strong>Usage:</strong> Load this model with: <code>ros2 run xacro xacro {modelName}.urdf</code>
      </div>
    </div>
  );
};

export { CodeBlock, ROS2NodeComponent, SimulationComponent, URDFModelComponent };