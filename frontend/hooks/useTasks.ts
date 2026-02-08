import { useState, useEffect } from 'react';
import { Task, CreateTaskData, UpdateTaskData } from '../types/task';
import { apiClient } from '../lib/api';

export const useTasks = (userId: string) => {
  const [tasks, setTasks] = useState<Task[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const fetchTasks = async () => {
    try {
      setLoading(true);
      const response: any[] = await apiClient.get<any[]>(`/api/${userId}/tasks`, userId);
      const transformedTasks = response?.map(task => ({
        id: task.id,
        userId: task.user_id,
        title: task.title,
        description: task.description,
        completed: task.completed,
        createdAt: task.created_at,
        updatedAt: task.updated_at
      })) || [];
      setTasks(transformedTasks);
    } catch (err) {
      setError('Failed to fetch tasks');
      console.error('Error fetching tasks:', err);
    } finally {
      setLoading(false);
    }
  };

  const createTask = async (taskData: CreateTaskData) => {
    try {
      const response: any = await apiClient.post<any>(`/api/${userId}/tasks`, taskData, userId);
      const newTask = {
        id: response.id,
        userId: response.user_id,
        title: response.title,
        description: response.description,
        completed: response.completed,
        createdAt: response.created_at,
        updatedAt: response.updated_at
      };
      setTasks(prev => [newTask, ...prev]);
      return newTask;
    } catch (err) {
      setError('Failed to create task');
      console.error('Error creating task:', err);
      throw err;
    }
  };

  const updateTask = async (taskId: string, taskData: UpdateTaskData) => {
    try {
      const response: any = await apiClient.put<any>(`/api/${userId}/tasks/${taskId}`, taskData, userId);
      const updatedTask = {
        id: response.id,
        userId: response.user_id,
        title: response.title,
        description: response.description,
        completed: response.completed,
        createdAt: response.created_at,
        updatedAt: response.updated_at
      };
      setTasks(prev => prev.map(task => task.id === taskId ? updatedTask : task));
      return updatedTask;
    } catch (err) {
      setError('Failed to update task');
      console.error('Error updating task:', err);
      throw err;
    }
  };

  const deleteTask = async (taskId: string) => {
    try {
      await apiClient.delete(`/api/${userId}/tasks/${taskId}`, userId);
      setTasks(prev => prev.filter(task => task.id !== taskId));
    } catch (err) {
      setError('Failed to delete task');
      console.error('Error deleting task:', err);
      throw err;
    }
  };

  const toggleTaskCompletion = async (taskId: string) => {
    try {
      const task = tasks.find(t => t.id === taskId);
      if (!task) throw new Error('Task not found');

      const response: any = await apiClient.patch<any>(`/api/${userId}/tasks/${taskId}`, {
        completed: !task.completed
      }, userId);

      const updatedTask = {
        id: response.id,
        userId: response.user_id,
        title: response.title,
        description: response.description,
        completed: response.completed,
        createdAt: response.created_at,
        updatedAt: response.updated_at
      };

      setTasks(prev => prev.map(t => t.id === taskId ? updatedTask : t));
      return updatedTask;
    } catch (err) {
      setError('Failed to toggle task completion');
      console.error('Error toggling task completion:', err);
      throw err;
    }
  };

  useEffect(() => {
    if (userId) {
      fetchTasks();
    }
  }, [userId]);

  return {
    tasks,
    loading,
    error,
    fetchTasks,
    createTask,
    updateTask,
    deleteTask,
    toggleTaskCompletion,
  };
};