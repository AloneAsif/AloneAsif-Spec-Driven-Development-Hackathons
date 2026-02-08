// Simple JWT decoder utility
export const decodeJWT = (token: string): any => {
  try {
    // Split the token into its parts
    const parts = token.split('.');

    if (parts.length !== 3) {
      throw new Error('Invalid token format');
    }

    // Decode the payload (second part)
    // Replace '-' with '+' and '_' with '/' and add padding if needed
    let payload = parts[1];
    payload = payload.replace(/-/g, '+').replace(/_/g, '/');

    // Add padding if needed
    const pad = 4 - (payload.length % 4);
    if (pad < 4) {
      payload += '='.repeat(pad);
    }

    // Decode from base64
    const decodedPayload = atob(payload);

    // Parse as JSON
    return JSON.parse(decodedPayload);
  } catch (error) {
    console.error('Error decoding JWT:', error);
    throw new Error('Failed to decode JWT token');
  }
};

// Get user ID from JWT token
export const getUserIdFromToken = (token: string): string | null => {
  try {
    const decoded = decodeJWT(token);
    return decoded.user_id || decoded.userId || decoded.sub || null;
  } catch (error) {
    console.error('Error extracting user ID from token:', error);
    return null;
  }
};