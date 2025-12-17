# Authentication Setup Guide

## Quick Start - With Authentication

### Backend Setup

```bash
cd backend

# Install dependencies (already updated)
pip install -r requirements.txt

# Run backend
python -m uvicorn src.app:app --reload
```

Backend will be available at: `http://localhost:8000`

### Frontend Setup

```bash
# Install dependencies
npm install

# Start development server
npm start
```

Frontend will be available at: `http://localhost:3000`

---

## Testing the Authentication

### Step 1: Create Account

1. Open http://localhost:3000
2. Click "Sign Up"
3. Fill in:
   - Full Name: John Doe
   - Email: john@example.com
   - Username: johndoe
   - Password: SecurePass123 (must have uppercase + number)
   - Confirm Password: SecurePass123

### Step 2: Fill Background Questionnaire

4. Select your experience level:
   - Software Background: Beginner/Intermediate/Advanced/Expert
   - Hardware Background: Beginner/Intermediate/Advanced/Expert

5. Select programming languages you know:
   - Python, C++, ROS, etc.

6. Select interest areas:
   - ROS, Kinematics, Control, Perception, Humanoid, etc.

7. Fill optional info:
   - Profession: Student/Engineer/Researcher
   - Organization: Your company/university
   - Years of Experience: 0-60

8. Learning preferences:
   - Learning Style: Visual/Text/Interactive/Hands-on
   - Learning Pace: Slow/Moderate/Fast

9. Click "Complete Profile"

### Step 3: Use Personalized Chatbot

10. Once logged in, you'll see the chat widget
11. Your responses will be personalized based on your profile
12. Click the user menu (top right) to see your profile

---

## API Endpoints

All authentication endpoints start with `/auth`

### Public Endpoints

```bash
# Signup
POST /auth/signup
Body: {email, username, password, full_name}
Response: {status, message, user_id}

# Signin
POST /auth/signin
Body: {email, password}
Response: {access_token, refresh_token, user}
```

### Protected Endpoints (require Bearer token)

```bash
# Get user profile
GET /auth/profile
Header: Authorization: Bearer <token>
Response: User details

# Save background
POST /auth/profile/background
Header: Authorization: Bearer <token>
Body: {software_background, hardware_background, ...}
Response: Updated user

# Get personalization data
GET /auth/personalization
Header: Authorization: Bearer <token>
Response: {personalized_content, recommendations}
```

---

## Environment Variables

### Backend (.env)

```env
# JWT Configuration
JWT_SECRET_KEY=your-super-secret-key
JWT_ALGORITHM=HS256
JWT_EXPIRE_MINUTES=1440
JWT_REFRESH_EXPIRE_DAYS=30

# OpenAI & Qdrant (existing)
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
```

### Frontend

```env
REACT_APP_API_URL=http://localhost:8000  # For local
# Or leave empty to auto-detect
```

---

## Features

✅ User signup with validation
✅ User signin with JWT
✅ Background questionnaire
✅ Personalized learning experience
✅ Profile management
✅ Session persistence
✅ Logout functionality
✅ User menu with quick info
✅ Learning recommendations

---

## Password Requirements

- Minimum 8 characters
- At least 1 uppercase letter
- At least 1 number

Example valid passwords:
- `SecurePass123`
- `MyPassword999`
- `Learning2024`

Invalid passwords:
- `password123` (no uppercase)
- `PASSWORD` (no number)
- `Pass1` (too short)

---

## Files Structure

```
backend/
├── src/
│   ├── models/user.py                 # User model
│   ├── services/
│   │   ├── auth_service.py            # Auth logic
│   │   └── personalization_service.py # Personalization
│   └── api/auth.py                    # Auth endpoints

src/
├── components/
│   ├── AuthForm.jsx                   # Auth forms
│   ├── AppWithAuth.jsx                # App wrapper
│   ├── UserMenu.jsx                   # User menu
│   └── ChatWidget.jsx                 # Chat (updated)
└── context/
    └── AuthContext.jsx                # Auth state
```

---

## Troubleshooting

### "Invalid password"
- Check: Min 8 chars, 1 uppercase, 1 number
- Example: `SecurePass123`

### "Email already registered"
- Use different email or login instead

### Token errors
- Clear localStorage: `localStorage.clear()`
- Refresh the page
- Sign in again

### Backend won't start
```bash
# Check Python installation
python --version  # Should be 3.8+

# Reinstall dependencies
pip install --upgrade -r requirements.txt

# Try different port
python -m uvicorn src.app:app --port 8001
```

---

## Test Accounts

For quick testing, you can use:

**Account 1**
- Email: `demo@example.com`
- Username: `demouser`
- Password: `DemoPass123`

**Account 2**
- Email: `test@example.com`
- Username: `testuser`
- Password: `TestPass123`

---

## Next Steps

1. Run local setup
2. Create account
3. Fill background questionnaire
4. Ask questions to see personalization in action
5. Notice how responses adapt to your level

---

## Support

- Full implementation docs: See `AUTH_IMPLEMENTATION.md`
- API docs: http://localhost:8000/api/docs
- Troubleshooting: Check AUTH_IMPLEMENTATION.md

Enjoy your personalized learning experience! 🚀
