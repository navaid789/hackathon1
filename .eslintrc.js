module.exports = {
  root: true,
  extends: [
    '@docusaurus',
    'prettier'
  ],
  plugins: ['prettier'],
  rules: {
    'prettier/prettier': 'error',
  },
  overrides: [
    {
      files: ['**/*.js', '**/*.jsx'],
      rules: {
        'no-console': 'warn',
        'no-unused-vars': 'error',
      },
    },
  ],
};