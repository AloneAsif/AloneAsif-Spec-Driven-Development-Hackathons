'use client';

import { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { User } from '../types/user';
import { isAuthenticated, getAuthToken, removeAuthToken, signIn as apiSignIn, signUp as apiSignUp, signOut as apiSignOut } from '../lib/auth';
import { getUserIdFromToken } from '../utils/jwt';

interface AuthContextType {
  user: User | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signUp: (email: string, name: string, password: string) => Promise<void>;
  signOut: () => Promise<void>;
  checkAuthStatus: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

  // Check auth status on initial load
  useEffect(() => {
    checkAuthStatus();
  }, []);

  const checkAuthStatus = async () => {
    setLoading(true);
    try {
      if (isAuthenticated()) {
        // In a real app, we would fetch user details from the API
        // For now, we'll create a mock user based on token
        const token = getAuthToken();
        if (token) {
          // Decode the token to get user info
          try {
            const userId = getUserIdFromToken(token);
            setUser({
              id: userId || 'unknown-user-id',
              email: 'mock@example.com',
              name: 'Mock User',
              createdAt: new Date().toISOString(),
              updatedAt: new Date().toISOString(),
            });
          } catch (decodeError) {
            console.error('Error decoding token:', decodeError);
            // Fallback to mock user if token decode fails
            setUser({
              id: 'mock-user-id',
              email: 'mock@example.com',
              name: 'Mock User',
              createdAt: new Date().toISOString(),
              updatedAt: new Date().toISOString(),
            });
          }
        }
      }
    } catch (error) {
      console.error('Error checking auth status:', error);
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  const signIn = async (email: string, password: string) => {
    setLoading(true);
    try {
      // Call the actual API sign-in function
      const result = await apiSignIn(email, password);

      // Update user state with the returned user data
      if (result.user) {
        setUser(result.user);
      }
    } catch (error) {
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const signUp = async (email: string, name: string, password: string) => {
    setLoading(true);
    try {
      // Call the actual API sign-up function
      const result = await apiSignUp(email, name, password);

      // Update user state with the returned user data
      if (result.user) {
        setUser(result.user);
      }
    } catch (error) {
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const signOut = async () => {
    setLoading(true);
    try {
      // Call the actual API sign-out function
      await apiSignOut();

      // Clear user state
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  const value: AuthContextType = {
    user,
    loading,
    signIn,
    signUp,
    signOut,
    checkAuthStatus,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};