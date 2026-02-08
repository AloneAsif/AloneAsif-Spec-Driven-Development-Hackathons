'use client';

import { useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { useTasks } from '../../hooks/useTasks';
import { useAuth } from '../../context/AuthContext';
import TaskForm from '../../components/TaskForm';
import TaskCard from '../../components/TaskCard';
import Navbar from '../../components/Navbar';

const TasksPage = () => {
  const { user, loading: authLoading } = useAuth();
  const { tasks, loading, error, createTask, updateTask, deleteTask, toggleTaskCompletion } = useTasks(user?.id || '');
  const router = useRouter();

  // Redirect if not authenticated
  useEffect(() => {
    if (!authLoading && !user) {
      router.push('/auth/signin');
    }
  }, [authLoading, user, router]);

  if (authLoading || !user) {
    return (
      <div className="min-h-screen bg-gray-50">
        <Navbar />
        <div className="py-12">
          <div className="container mx-auto px-4">
            <div className="max-w-3xl mx-auto">
              <div className="text-center">
                <h1 className="text-3xl font-bold text-gray-800 mb-2">Loading...</h1>
                <p className="text-gray-600">Please wait while we load your tasks.</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  const handleTaskCreated = (newTask: any) => {
    // The custom hook manages the state
  };

  const handleTaskUpdated = (updatedTask: any) => {
    // The custom hook manages the state
  };

  const handleTaskDeleted = async (taskId: string) => {
    try {
      await deleteTask(taskId);
    } catch (err) {
      console.error('Error deleting task:', err);
    }
  };

  const handleToggleCompletion = async (taskId: string) => {
    try {
      await toggleTaskCompletion(taskId);
    } catch (err) {
      console.error('Error toggling task completion:', err);
    }
  };

  return (
    <div className="min-h-screen bg-gray-50">
      <Navbar />
      <div className="py-12">
        <div className="container mx-auto px-4">
          <div className="max-w-3xl mx-auto">
            <div className="text-center mb-8">
              <h1 className="text-3xl font-bold text-gray-800">My Tasks</h1>
              <p className="text-gray-600 mt-2">Manage your personal tasks efficiently</p>
            </div>

            {error && (
              <div className="mb-6 p-4 bg-red-100 text-red-700 rounded-lg">
                {error}
              </div>
            )}

            <TaskForm userId={user.id} onTaskCreated={handleTaskCreated} />

            {tasks.length > 0 ? (
              <div>
                <h2 className="text-xl font-semibold text-gray-700 mb-4">
                  Your Tasks ({tasks.length})
                </h2>
                <div className="bg-white rounded-lg shadow overflow-hidden">
                  {tasks.map((task) => (
                    <TaskCard
                      key={task.id}
                      task={task}
                      onUpdate={handleTaskUpdated}
                      onDelete={handleTaskDeleted}
                    />
                  ))}
                </div>
              </div>
            ) : (
              <div className="text-center py-12 bg-white rounded-lg shadow">
                <h3 className="text-xl font-medium text-gray-700 mb-2">No tasks yet</h3>
                <p className="text-gray-600">Add your first task using the form above!</p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default TasksPage;