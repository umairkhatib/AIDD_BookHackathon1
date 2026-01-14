# Data Model: Physical AI & Humanoid Robotics Course

**Feature**: 001-physical-ai-humanoid
**Date**: 2026-01-09
**Status**: Completed

## Overview

This document defines the data structures and entities for the Physical AI & Humanoid Robotics course. Since this is primarily a documentation and educational platform, the data model focuses on content organization and educational resources.

## Core Entities

### 1. CourseModule
**Description**: Represents a major section of the course with learning objectives and content

**Fields**:
- `id`: Unique identifier for the module (string, required)
- `title`: Display title of the module (string, required)
- `description`: Brief description of the module content (string, required)
- `learningOutcomes`: Array of learning outcomes for the module (array of strings, required)
- `order`: Sequential position in the course (integer, required)
- `duration`: Estimated completion time in hours (float, required)
- `prerequisites`: Array of prerequisite module IDs (array of strings, optional)
- `topics`: Array of topic identifiers covered in the module (array of strings, required)
- `exercises`: Array of exercise identifiers associated with the module (array of strings, required)

**Validation Rules**:
- `id` must be unique across all modules
- `order` must be positive integer
- `duration` must be greater than 0
- `learningOutcomes` must contain at least one outcome

### 2. Topic
**Description**: A specific subject or concept within a module

**Fields**:
- `id`: Unique identifier for the topic (string, required)
- `moduleId`: Reference to the parent module (string, required)
- `title`: Display title of the topic (string, required)
- `description`: Brief description of the topic (string, required)
- `content`: Main content of the topic in MDX format (string, required)
- `order`: Sequential position within the module (integer, required)
- `difficulty`: Difficulty level (enum: beginner, intermediate, advanced, required)
- `estimatedTime`: Estimated reading/learning time in minutes (integer, required)
- `resources`: Array of resource identifiers (array of strings, optional)
- `prerequisites`: Array of prerequisite topic IDs (array of strings, optional)

**Validation Rules**:
- `moduleId` must reference an existing module
- `order` must be positive integer
- `estimatedTime` must be greater than 0
- `difficulty` must be one of the allowed values

### 3. Exercise
**Description**: Hands-on activity or assignment for students

**Fields**:
- `id`: Unique identifier for the exercise (string, required)
- `moduleId`: Reference to the parent module (string, required)
- `title`: Display title of the exercise (string, required)
- `description`: Brief description of the exercise (string, required)
- `instructions`: Detailed instructions in MDX format (string, required)
- `solution`: Solution or reference implementation (string, optional)
- `type`: Exercise category (enum: coding, simulation, analysis, conceptual, required)
- `complexity`: Complexity level (enum: basic, intermediate, advanced, required)
- `estimatedDuration`: Estimated completion time in minutes (integer, required)
- `dependencies`: Array of required tools or resources (array of strings, optional)
- `validationCriteria`: Array of criteria for successful completion (array of strings, required)

**Validation Rules**:
- `moduleId` must reference an existing module
- `estimatedDuration` must be greater than 0
- `type` and `complexity` must be one of the allowed values
- `validationCriteria` must contain at least one criterion

### 4. Resource
**Description**: Supplementary material or asset for educational content

**Fields**:
- `id`: Unique identifier for the resource (string, required)
- `title`: Display title of the resource (string, required)
- `type`: Resource category (enum: video, document, code, simulation, dataset, required)
- `url`: Location of the resource (string, required)
- `description`: Brief description of the resource (string, optional)
- `size`: Size of the resource in MB (float, optional)
- `tags`: Array of tags for categorization (array of strings, optional)
- `relatedTopics`: Array of topic IDs this resource relates to (array of strings, optional)
- `license`: License information (string, required)

**Validation Rules**:
- `url` must be a valid URL
- `type` must be one of the allowed values
- `size` must be greater than 0 if provided

### 5. SimulationScenario
**Description**: Configuration for a simulation environment or experiment

**Fields**:
- `id`: Unique identifier for the scenario (string, required)
- `moduleId`: Reference to the parent module (string, required)
- `title`: Display title of the scenario (string, required)
- `description`: Brief description of the scenario (string, required)
- `environment`: Target simulation environment (enum: gazebo, isaac_sim, unity, required)
- `configuration`: Path to configuration files (string, required)
- `robotModel`: URDF or model description to use (string, required)
- `objectives`: Array of learning objectives for the scenario (array of strings, required)
- `steps`: Array of sequential steps for the scenario (array of strings, required)
- `expectedOutcome`: Description of expected results (string, required)
- `troubleshooting`: Common issues and solutions (string, optional)

**Validation Rules**:
- `moduleId` must reference an existing module
- `environment` must be one of the allowed values
- `objectives` must contain at least one objective

### 6. Assessment
**Description**: Evaluation component for measuring student understanding

**Fields**:
- `id`: Unique identifier for the assessment (string, required)
- `moduleId`: Reference to the parent module (string, required)
- `title`: Display title of the assessment (string, required)
- `type`: Assessment format (enum: quiz, project, peer_review, practical, required)
- `questions`: Array of question objects (array of objects, required for quizzes)
- `rubric`: Evaluation criteria (string, required for projects)
- `passingScore`: Minimum score required to pass (float, required)
- `timeLimit`: Time limit in minutes (integer, optional)
- `attemptsAllowed`: Number of attempts permitted (integer, optional)
- `feedback`: Feedback mechanism description (string, optional)

**Validation Rules**:
- `moduleId` must reference an existing module
- `type` must be one of the allowed values
- `passingScore` must be between 0 and 100
- `timeLimit` and `attemptsAllowed` must be greater than 0 if provided

## Relationships

```
CourseModule (1) ←→ (Many) Topic
CourseModule (1) ←→ (Many) Exercise
CourseModule (1) ←→ (Many) SimulationScenario
CourseModule (1) ←→ (Many) Assessment

Topic (Many) ←→ (Many) Resource
Exercise (1) ←→ (Many) Resource
SimulationScenario (1) ←→ (Many) Resource
```

## State Transitions

### Module Progression
- `not_started` → `in_progress` → `completed`
- `completed` → `reviewed` (optional)

### Exercise Status
- `not_attempted` → `in_progress` → `submitted` → `graded`
- `graded` → `reviewed` (optional)

## Indexes and Access Patterns

### Common Queries
1. Retrieve all modules in course order
2. Get all topics within a specific module
3. Find all exercises of a certain type within a module
4. Search resources by tags
5. Get simulation scenarios by environment type

### Recommended Indexes
- CourseModule.order
- Topic.moduleId + Topic.order
- Exercise.moduleId + Exercise.type
- Resource.tags
- SimulationScenario.environment