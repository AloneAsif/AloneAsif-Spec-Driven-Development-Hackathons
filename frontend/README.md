# Todo Application Frontend

A modern, responsive todo application built with Next.js, TypeScript, and Tailwind CSS.

## Features

- User authentication (sign up/sign in)
- Task management (create, read, update, delete)
- Task completion toggling
- Responsive design for mobile and desktop
- Dark/light mode support
- Real-time updates

## Tech Stack

- Next.js 16+ with App Router
- TypeScript
- Tailwind CSS for styling
- Better Auth for authentication
- React Icons for icons

## Getting Started

### Prerequisites

- Node.js (v18.17.0 or higher)
- npm or yarn package manager

### Installation

1. Clone the repository
2. Navigate to the frontend directory: `cd frontend`
3. Install dependencies: `npm install` or `yarn install`
4. Create a `.env.local` file in the frontend directory:
   ```
   NEXT_PUBLIC_API_BASE_URL=http://localhost:3000/api
   NEXT_PUBLIC_BETTER_AUTH_URL=http://localhost:3000
   BETTER_AUTH_SECRET=your-secret-key-here
   ```

### Running the Application

```bash
# Development mode
npm run dev
# or
yarn dev
```

The application will be available at [http://localhost:3000](http://localhost:3000)

### Building for Production

```bash
# Build the application
npm run build
# or
yarn build
```

### Starting Production Server

```bash
# Start production server
npm start
# or
yarn start
```

## Project Structure

```
frontend/
├── app/                 # Next.js App Router pages
│   ├── layout.tsx      # Root layout component
│   ├── page.tsx        # Home page
│   ├── tasks/          # Task management pages
│   └── auth/           # Authentication pages
├── components/         # Reusable React components
│   ├── TaskCard.tsx    # Task display component
│   ├── TaskForm.tsx    # Task creation form
│   ├── Navbar.tsx      # Navigation component
│   └── ui/             # Reusable UI components
├── lib/               # Utilities and API clients
│   ├── api.ts          # API client with JWT handling
│   └── auth.ts         # Authentication utilities
├── types/             # TypeScript type definitions
├── hooks/             # Custom React hooks
├── utils/             # Utility functions
├── styles/            # Global styles
├── public/            # Static assets
├── package.json       # Project dependencies
└── tsconfig.json      # TypeScript configuration
```

## Environment Variables

- `NEXT_PUBLIC_API_BASE_URL`: Base URL for the API server
- `NEXT_PUBLIC_BETTER_AUTH_URL`: Base URL for Better Auth server
- `BETTER_AUTH_SECRET`: Secret key for JWT authentication

## API Integration

The application uses a centralized API client located at `lib/api.ts` that automatically includes JWT tokens in requests.