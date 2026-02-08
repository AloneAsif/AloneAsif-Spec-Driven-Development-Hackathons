'use client';

import { Task } from '../types/task';
import { useState } from 'react';
import { apiClient } from '../lib/api';
import { FiEdit2, FiTrash2, FiCheck, FiX } from 'react-icons/fi';

interface TaskCardProps {
  task: Task;
  onUpdate: (updatedTask: Task) => void;
  onDelete: (taskId: string) => void;
}

const TaskCard: React.FC<TaskCardProps> = ({ task, onUpdate, onDelete }) => {
  const [isEditing, setIsEditing] = useState(false);
  const [title, setTitle] = useState(task.title);
  const [description, setDescription] = useState(task.description || '');
  const [isUpdating, setIsUpdating] = useState(false);

  const handleToggleComplete = async () => {
    try {
      const response: any = await apiClient.patch<any>(`/api/${task.userId}/tasks/${task.id}`, {
        completed: !task.completed,
      }, task.userId);
      const updatedTask = {
        id: response.id,
        userId: response.user_id,
        title: response.title,
        description: response.description,
        completed: response.completed,
        createdAt: response.created_at,
        updatedAt: response.updated_at
      };
      onUpdate(updatedTask);
    } catch (error) {
      console.error('Failed to update task:', error);
    }
  };

  const handleSave = async () => {
    setIsUpdating(true);
    try {
      const response: any = await apiClient.put<any>(`/api/${task.userId}/tasks/${task.id}`, {
        title,
        description,
        completed: task.completed,
      }, task.userId);
      const updatedTask = {
        id: response.id,
        userId: response.user_id,
        title: response.title,
        description: response.description,
        completed: response.completed,
        createdAt: response.created_at,
        updatedAt: response.updated_at
      };
      onUpdate(updatedTask);
      setIsEditing(false);
    } catch (error) {
      console.error('Failed to update task:', error);
    } finally {
      setIsUpdating(false);
    }
  };

  const handleCancel = () => {
    setTitle(task.title);
    setDescription(task.description || '');
    setIsEditing(false);
  };

  return (
    <div className={`border rounded-lg p-4 mb-3 fade-in transition-all duration-300 ${task.completed ? 'bg-green-50 dark:bg-green-900/20 border-green-200 dark:border-green-800' : 'bg-white dark:bg-gray-800'}`}>
      {isEditing ? (
        <div className="space-y-3">
          <input
            type="text"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            className="w-full p-2 border rounded dark:bg-gray-700 dark:border-gray-600 dark:text-white text-gray-900"
            placeholder="Task title"
          />
          <textarea
            value={description}
            onChange={(e) => setDescription(e.target.value)}
            className="w-full p-2 border rounded dark:bg-gray-700 dark:border-gray-600 dark:text-white text-gray-900"
            placeholder="Task description"
          />
          <div className="flex space-x-2">
            <button
              onClick={handleSave}
              disabled={isUpdating}
              className="bg-primary-500 text-white px-4 py-2 rounded flex items-center dark:bg-primary-600"
            >
              {isUpdating ? 'Saving...' : 'Save'}
            </button>
            <button
              onClick={handleCancel}
              className="bg-gray-300 px-4 py-2 rounded flex items-center dark:bg-gray-600 dark:text-white"
            >
              Cancel
            </button>
          </div>
        </div>
      ) : (
        <>
          <div className="flex justify-between items-start">
            <div>
              <h3 className={`font-medium ${task.completed ? 'line-through text-gray-500 dark:text-gray-400' : 'text-gray-800 dark:text-gray-100'}`}>
                {task.title}
              </h3>
              {task.description && (
                <p className={`mt-1 ${task.completed ? 'line-through' : ''} text-gray-600 dark:text-gray-300`}>
                  {task.description}
                </p>
              )}
            </div>
            <div className="flex space-x-2">
              <button
                onClick={() => setIsEditing(true)}
                className="text-blue-500 hover:text-blue-700 dark:text-blue-400 dark:hover:text-blue-300"
                aria-label="Edit task"
              >
                <FiEdit2 />
              </button>
              <button
                onClick={() => onDelete(String(task.id))}
                className="text-red-500 hover:text-red-700 dark:text-red-400 dark:hover:text-red-300"
                aria-label="Delete task"
              >
                <FiTrash2 />
              </button>
            </div>
          </div>
          <div className="mt-3 flex justify-between items-center">
            <div className="text-sm text-gray-500 dark:text-gray-400">
              Created: {new Date(task.createdAt).toLocaleDateString()}
            </div>
            <button
              onClick={handleToggleComplete}
              className={`p-2 rounded-full ${
                task.completed
                  ? 'bg-green-100 text-green-600 dark:bg-green-900/30 dark:text-green-400'
                  : 'bg-gray-100 text-gray-600 dark:bg-gray-700 dark:text-gray-300'
              }`}
              aria-label={task.completed ? 'Mark as incomplete' : 'Mark as complete'}
            >
              {task.completed ? <FiX /> : <FiCheck />}
            </button>
          </div>
        </>
      )}
    </div>
  );
};

export default TaskCard;