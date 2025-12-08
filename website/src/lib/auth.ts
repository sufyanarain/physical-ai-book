// Authentication client for backend API

const BACKEND_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://physical-ai-backend-production-b62f.up.railway.app';

// API client for our backend
export const api = {
  signup: async (data: {
    email: string;
    name: string;
    password: string;
    software_experience: string;
    hardware_experience: string;
    learning_goals?: string;
  }) => {
    const response = await fetch(`${BACKEND_URL}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });
    
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Signup failed');
    }
    
    return response.json();
  },
  
  login: async (data: { email: string; password: string }) => {
    const response = await fetch(`${BACKEND_URL}/auth/login`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });
    
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.detail || 'Login failed');
    }
    
    return response.json();
  },
  
  getMe: async (token: string) => {
    const response = await fetch(`${BACKEND_URL}/auth/me`, {
      headers: {
        'Authorization': `Bearer ${token}`,
      },
    });
    
    if (!response.ok) {
      throw new Error('Failed to get user');
    }
    
    return response.json();
  },
};
