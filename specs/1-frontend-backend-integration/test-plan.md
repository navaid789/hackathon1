# End-to-End Test Suite: Frontend-Backend Integration

## Test Plan

This test suite validates the frontend-backend integration according to the success criteria defined in the specification.

### Success Criteria Validation

#### SC-001: Connection Establishment
**Test**: Verify frontend can establish connection to local FastAPI backend without CORS errors
**Method**:
1. Start backend server on localhost:8000
2. Start frontend server on localhost:3000
3. Make API request from frontend to backend
4. Verify no CORS errors in browser console

#### SC-002: Response Time
**Test**: Verify user queries return valid agent responses within 10 seconds under normal conditions
**Method**:
1. Submit query to backend
2. Measure time from request to response
3. Verify response time < 10 seconds

#### SC-003: Context Accuracy
**Test**: Verify selected text is correctly passed as context and reflected in agent responses with 95% accuracy
**Method**:
1. Select specific text from book content
2. Submit query referencing that text
3. Verify response addresses the selected context
4. Test with multiple text selections

#### SC-004: Connection Reliability
**Test**: Verify 90% of user sessions experience zero connection failures during normal usage
**Method**:
1. Simulate multiple user sessions
2. Track connection success/failure rates
3. Verify >90% success rate

### Edge Case Testing

#### T044: Very Long Text Selections
**Test**: Verify system handles text selections up to 5000 characters
**Method**:
1. Select text of maximum allowed length (5000 chars)
2. Submit query with context
3. Verify response is generated successfully

#### T044: Complex Queries
**Test**: Verify system handles complex multi-part queries
**Method**:
1. Submit complex queries with multiple parts
2. Verify response addresses all parts of query
3. Test with nested questions and follow-ups

## Test Implementation

### Manual Test Script
```bash
# 1. Start backend
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
uvicorn main:app --reload --port 8000

# 2. In a new terminal, start frontend
cd ..
npm run start

# 3. Manual testing steps:
# - Navigate to book interface
# - Select text from book content
# - Enter a query related to selected text
# - Verify response includes relevant information
# - Check browser console for errors
```

### Automated Test Script (Example)
```javascript
// test-integration.js
const { test } = require('node:test');
const assert = require('node:assert');

// Test API connectivity
test('should connect to backend API', async () => {
  const response = await fetch('http://localhost:8000/health');
  assert.strictEqual(response.status, 200);
});

// Test query functionality with context
test('should handle query with context', async () => {
  const queryData = {
    query: "What are the main themes?",
    context: "The author explores themes of identity and belonging..."
  };

  const response = await fetch('http://localhost:8000/query', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(queryData)
  });

  const result = await response.json();
  assert.ok(response.ok);
  assert.ok(result.response);
  assert.ok(result.thread_id);
  assert.ok(result.query_id);
});
```

## Test Execution Steps

1. **Environment Setup**:
   - Ensure backend is running on localhost:8000
   - Ensure frontend is running on localhost:3000
   - Verify all required environment variables are set

2. **Basic Functionality Test**:
   - Navigate to the book interface
   - Verify chatbot UI is visible and functional
   - Test basic query without context
   - Test query with selected text context

3. **Success Criteria Validation**:
   - Time API responses to validate SC-002
   - Test context passing to validate SC-003
   - Check for connection errors to validate SC-004

4. **Edge Case Testing**:
   - Test with maximum length text selection
   - Test with complex queries
   - Test error handling scenarios

## Expected Results

- All API requests succeed without CORS errors
- Response times are under 10 seconds
- Context is properly passed and reflected in responses
- No connection failures during normal usage
- Edge cases are handled gracefully

## Test Report Template

```
Test Execution Date: [DATE]
Environment: [Local Development]
Backend Version: [VERSION]
Frontend Version: [VERSION]

SC-001 (Connection): [PASS/FAIL] - [Details]
SC-002 (Response Time): [PASS/FAIL] - [Avg: X.XXs]
SC-003 (Context Accuracy): [PASS/FAIL] - [Accuracy: XX%]
SC-004 (Reliability): [PASS/FAIL] - [Success Rate: XX%]

Edge Cases:
- Long text selection: [PASS/FAIL]
- Complex queries: [PASS/FAIL]
- Error handling: [PASS/FAIL]

Issues Found:
- [List any issues discovered]

Recommendations:
- [Any recommendations for improvements]
```