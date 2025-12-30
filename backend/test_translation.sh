#!/bin/bash

TOKEN="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiIxIiwiZW1haWwiOiJ0ZXN0QGV4YW1wbGUuY29tIiwidXNlcm5hbWUiOiJ0ZXN0dXNlciIsImV4cCI6MTc2NjQwODg3OH0.w9e6pmVQzAjRn58dFlJ3rA8ZL7c4TgMlAemXob8sODY"

curl -s -X POST "http://localhost:8000/translation/translate" \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{"content":"<p>Education is very important.</p>","chapter_id":"test-chapter-1","target_language":"urdu"}' \
  | python -m json.tool
