#!/usr/bin/env node

/**
 * Citation validation script for Physical AI & Humanoid Robotics textbook
 * Checks that all technical claims are properly cited with peer-reviewed sources or official documentation
 */

const fs = require('fs');
const path = require('path');

// Define patterns that indicate technical claims requiring citations
const TECHNICAL_CLAIM_PATTERNS = [
  /\b(claim|assert|state|show|demonstrate|prove|indicate|suggest|find|discover|reveal)\b/i,
  /\b(according to|based on|from|in|study|research|paper|article)\b/i,
  /\b(achieve|reach|obtain|attain|realize)\b.*?\d+(%|x|fps|ms|seconds|minutes|hours)/i,
  /\b(benchmark|performance|accuracy|precision|recall|f1-score|success rate)\b/i,
  /\b(hardware|requirement|specification|minimum|recommended)\b/i,
  /\b(ROS 2|Isaac Sim|Gazebo|Unity|Ubuntu|RTX|Jetson)\b/i,
  /\b(sim-to-real|transfer learning|reinforcement learning|VLA|vision-language-action)\b/i
];

// Define acceptable citation formats
const CITATION_PATTERNS = [
  /\[@\w+\]/, // [@author2022]
  /\[(\d+)\]/, // [1]
  /\[([A-Z][a-z]+ et al\., \d{4})\]/, // [Smith et al., 2022]
  /references\.md#\w+/, // links to references.md
  /see\s+\w+\s+\d+/i, // "see Chapter 3"
];

function validateCitationsInFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const lines = content.split('\n');
  const issues = [];

  for (let i = 0; i < lines.length; i++) {
    const line = lines[i];
    const lineNumber = i + 1;

    // Check if line contains technical claim but no citation
    if (hasTechnicalClaim(line) && !hasCitation(line)) {
      issues.push({
        file: filePath,
        line: lineNumber,
        content: line.trim(),
        issue: 'Technical claim found without citation'
      });
    }
  }

  return issues;
}

function hasTechnicalClaim(line) {
  return TECHNICAL_CLAIM_PATTERNS.some(pattern => pattern.test(line));
}

function hasCitation(line) {
  return CITATION_PATTERNS.some(pattern => pattern.test(line));
}

function validateAllMarkdownFiles() {
  const docsDir = path.join(__dirname, '..', 'docs');
  const issues = [];

  // Walk through docs directory and validate all markdown files
  function walkDir(dir) {
    const files = fs.readdirSync(dir);

    for (const file of files) {
      const filePath = path.join(dir, file);
      const stat = fs.statSync(filePath);

      if (stat.isDirectory()) {
        walkDir(filePath);
      } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
        const fileIssues = validateCitationsInFile(filePath);
        issues.push(...fileIssues);
      }
    }
  }

  walkDir(docsDir);
  return issues;
}

function main() {
  console.log('Validating citations in Physical AI & Humanoid Robotics textbook...');

  const issues = validateAllMarkdownFiles();

  if (issues.length === 0) {
    console.log('✅ All technical claims are properly cited!');
    process.exit(0);
  } else {
    console.log(`❌ Found ${issues.length} citation issues:`);
    console.log('');

    for (const issue of issues) {
      console.log(`File: ${issue.file}`);
      console.log(`Line ${issue.line}: ${issue.content}`);
      console.log(`Issue: ${issue.issue}`);
      console.log('');
    }

    process.exit(1);
  }
}

if (require.main === module) {
  main();
}