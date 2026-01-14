import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

/**
 * VLA Interactive Demo Component
 * Provides an interactive demonstration of Vision-Language-Action systems
 */
const VLAInteractiveDemo = () => {
  const [command, setCommand] = useState('');
  const [response, setResponse] = useState('');
  const [isProcessing, setIsProcessing] = useState(false);
  const [robotState, setRobotState] = useState({
    position: { x: 0, y: 0 },
    orientation: 0,
    status: 'idle'
  });
  const [history, setHistory] = useState([]);

  // Simulated robot actions
  const robotActions = [
    'move_forward',
    'move_backward',
    'turn_left',
    'turn_right',
    'go_to_kitchen',
    'go_to_bedroom',
    'go_to_living_room',
    'stop',
    'pick_up_object',
    'place_object'
  ];

  // Simulated command interpretations
  const interpretCommand = (cmd) => {
    cmd = cmd.toLowerCase();

    if (cmd.includes('forward') || cmd.includes('ahead')) return 'move_forward';
    if (cmd.includes('back') || cmd.includes('backward')) return 'move_backward';
    if (cmd.includes('left')) return 'turn_left';
    if (cmd.includes('right')) return 'turn_right';
    if (cmd.includes('kitchen')) return 'go_to_kitchen';
    if (cmd.includes('bedroom')) return 'go_to_bedroom';
    if (cmd.includes('living room') || cmd.includes('living_room')) return 'go_to_living_room';
    if (cmd.includes('stop') || cmd.includes('halt')) return 'stop';
    if (cmd.includes('pick up') || cmd.includes('grasp') || cmd.includes('take')) return 'pick_up_object';
    if (cmd.includes('place') || cmd.includes('put down')) return 'place_object';

    // If no specific action found, try to extract location
    for (const action of robotActions) {
      if (cmd.includes(action.replace('_', ' '))) {
        return action;
      }
    }

    return null;
  };

  // Simulated action execution
  const executeAction = (action) => {
    const newRobotState = { ...robotState };

    switch (action) {
      case 'move_forward':
        newRobotState.position.x += 1;
        newRobotState.status = 'moving';
        break;
      case 'move_backward':
        newRobotState.position.x -= 1;
        newRobotState.status = 'moving';
        break;
      case 'turn_left':
        newRobotState.orientation += Math.PI / 2;
        newRobotState.status = 'turning';
        break;
      case 'turn_right':
        newRobotState.orientation -= Math.PI / 2;
        newRobotState.status = 'turning';
        break;
      case 'go_to_kitchen':
        newRobotState.position = { x: 4, y: 3 };
        newRobotState.status = 'navigating';
        break;
      case 'go_to_bedroom':
        newRobotState.position = { x: -3, y: 2 };
        newRobotState.status = 'navigating';
        break;
      case 'go_to_living_room':
        newRobotState.position = { x: 0, y: 0 };
        newRobotState.status = 'navigating';
        break;
      case 'stop':
        newRobotState.status = 'idle';
        break;
      case 'pick_up_object':
        newRobotState.status = 'manipulating';
        break;
      case 'place_object':
        newRobotState.status = 'manipulating';
        break;
      default:
        newRobotState.status = 'idle';
    }

    setRobotState(newRobotState);

    // Return a response based on the action
    const responses = {
      'move_forward': 'Moving forward 1 meter',
      'move_backward': 'Moving backward 1 meter',
      'turn_left': 'Turning left 90 degrees',
      'turn_right': 'Turning right 90 degrees',
      'go_to_kitchen': 'Navigating to kitchen at coordinates (4, 3)',
      'go_to_bedroom': 'Navigating to bedroom at coordinates (-3, 2)',
      'go_to_living_room': 'Navigating to living room at coordinates (0, 0)',
      'stop': 'Stopping robot movement',
      'pick_up_object': 'Attempting to pick up object',
      'place_object': 'Placing object down',
      'unknown': 'Command not recognized. Available commands: move forward, turn left, go to kitchen, etc.'
    };

    return responses[action] || responses.unknown;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!command.trim()) return;

    setIsProcessing(true);

    // Add to history
    const newEntry = { command, timestamp: new Date().toLocaleTimeString() };
    setHistory(prev => [newEntry, ...prev.slice(0, 9)]); // Keep last 10 entries

    // Simulate processing delay
    setTimeout(() => {
      const interpretedAction = interpretCommand(command);
      let responseText;

      if (interpretedAction) {
        responseText = executeAction(interpretedAction);
      } else {
        responseText = 'Command not understood. Try commands like "move forward", "go to kitchen", "turn left", etc.';
      }

      setResponse(responseText);
      setIsProcessing(false);
    }, 1000);
  };

  return (
    <div style={{
      border: '1px solid #ddd',
      borderRadius: '8px',
      padding: '20px',
      margin: '20px 0',
      backgroundColor: '#f9f9f9'
    }}>
      <h3>Vision-Language-Action Interactive Demo</h3>

      <div style={{ marginBottom: '20px' }}>
        <p>Enter a natural language command for the robot:</p>
        <form onSubmit={handleSubmit}>
          <input
            type="text"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="e.g., 'Move forward', 'Go to kitchen', 'Turn left'"
            style={{
              width: '70%',
              padding: '8px',
              marginRight: '10px',
              border: '1px solid #ccc',
              borderRadius: '4px'
            }}
            disabled={isProcessing}
          />
          <button
            type="submit"
            disabled={isProcessing || !command.trim()}
            style={{
              padding: '8px 16px',
              backgroundColor: isProcessing ? '#ccc' : '#007cba',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: isProcessing || !command.trim() ? 'not-allowed' : 'pointer'
            }}
          >
            {isProcessing ? 'Processing...' : 'Send Command'}
          </button>
        </form>
      </div>

      {response && (
        <div style={{
          padding: '10px',
          backgroundColor: '#e8f4fd',
          borderRadius: '4px',
          marginBottom: '20px'
        }}>
          <strong>Robot Response:</strong> {response}
        </div>
      )}

      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        <div style={{ flex: '1', marginRight: '20px' }}>
          <h4>Robot State</h4>
          <div>
            <p><strong>Position:</strong> ({robotState.position.x}, {robotState.position.y})</p>
            <p><strong>Orientation:</strong> {(robotState.orientation * 180 / Math.PI).toFixed(1)}°</p>
            <p><strong>Status:</strong> {robotState.status}</p>
          </div>

          <h4>Available Commands</h4>
          <ul style={{ fontSize: '14px' }}>
            <li>Move forward/backward</li>
            <li>Turn left/right</li>
            <li>Go to kitchen/bedroom/living room</li>
            <li>Stop</li>
            <li>Pick up object</li>
            <li>Place object</li>
          </ul>
        </div>

        <div style={{ flex: '1' }}>
          <h4>Command History</h4>
          <div style={{
            maxHeight: '200px',
            overflowY: 'auto',
            border: '1px solid #ccc',
            padding: '10px',
            backgroundColor: 'white'
          }}>
            {history.length > 0 ? (
              <ul style={{ listStyle: 'none', padding: 0, margin: 0 }}>
                {history.map((entry, index) => (
                  <li key={index} style={{
                    borderBottom: '1px solid #eee',
                    padding: '5px 0',
                    fontSize: '14px'
                  }}>
                    <strong>{entry.timestamp}:</strong> "{entry.command}"
                  </li>
                ))}
              </ul>
            ) : (
              <p style={{ fontStyle: 'italic', color: '#888' }}>No commands yet</p>
            )}
          </div>
        </div>
      </div>

      <div style={{ marginTop: '20px', fontSize: '14px', color: '#666' }}>
        <p><strong>Note:</strong> This is a simulation of a Vision-Language-Action system. In a real implementation,
        the system would process visual input, interpret natural language commands using LLMs,
        and execute actions on a physical or simulated robot.</p>
      </div>
    </div>
  );
};

export default VLAInteractiveDemo;