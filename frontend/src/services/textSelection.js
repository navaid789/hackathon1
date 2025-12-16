// frontend/src/services/textSelection.js
// Utility for capturing text selection in the Docusaurus book interface

export class TextSelectionService {
  constructor() {
    this.lastSelection = null;
    this.selectionListeners = [];
  }

  // Get the currently selected text
  getSelectedText() {
    const selection = window.getSelection ? window.getSelection() : document.selection;
    if (selection) {
      const text = selection.toString().trim();
      return text;
    }
    return '';
  }

  // Get detailed selection information
  getSelectionInfo() {
    const selection = window.getSelection ? window.getSelection() : document.selection;

    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    const range = selection.rangeCount > 0 ? selection.getRangeAt(0) : null;

    if (!range) {
      return {
        text: selection.toString().trim(),
        context: null,
        position: null
      };
    }

    // Get surrounding context
    const startContainer = range.startContainer;
    const startOffset = range.startOffset;

    // Try to get parent element for context
    let contextElement = null;
    if (startContainer.nodeType === Node.TEXT_NODE) {
      contextElement = startContainer.parentElement;
    } else {
      contextElement = startContainer;
    }

    // Get the selected text
    const selectedText = selection.toString().trim();

    // Get additional context from the surrounding area
    let context = null;
    if (contextElement) {
      // Get the closest heading or section title as context
      const closestHeading = contextElement.closest('h1, h2, h3, h4, h5, h6');
      if (closestHeading) {
        context = {
          heading: closestHeading.textContent.trim(),
          elementTag: contextElement.tagName,
          elementId: contextElement.id || null,
          elementClass: contextElement.className || null
        };
      }
    }

    // Get cursor position information
    const rect = range.getBoundingClientRect();
    const position = {
      x: rect.left + window.scrollX,
      y: rect.top + window.scrollY,
      width: rect.width,
      height: rect.height
    };

    return {
      text: selectedText,
      context: context,
      position: position,
      range: range
    };
  }

  // Validate if the selected text meets requirements
  validateSelection(selectionInfo) {
    if (!selectionInfo || !selectionInfo.text) {
      return {
        isValid: false,
        errors: ['No text selected']
      };
    }

    if (selectionInfo.text.length > 5000) {
      return {
        isValid: false,
        errors: ['Selected text exceeds 5000 characters limit']
      };
    }

    if (selectionInfo.text.trim().length === 0) {
      return {
        isValid: false,
        errors: ['Selected text is empty']
      };
    }

    return {
      isValid: true,
      errors: []
    };
  }

  // Add event listener for selection changes
  addSelectionListener(callback) {
    if (typeof callback === 'function') {
      this.selectionListeners.push(callback);

      // Add the event listener to the document
      document.addEventListener('mouseup', this.handleSelectionChange);
      document.addEventListener('keyup', this.handleSelectionChange);
    }
  }

  // Remove event listener for selection changes
  removeSelectionListener(callback) {
    const index = this.selectionListeners.indexOf(callback);
    if (index > -1) {
      this.selectionListeners.splice(index, 1);
    }

    // If no more listeners, remove the event listeners
    if (this.selectionListeners.length === 0) {
      document.removeEventListener('mouseup', this.handleSelectionChange);
      document.removeEventListener('keyup', this.handleSelectionChange);
    }
  }

  // Internal method to handle selection changes
  handleSelectionChange = () => {
    const selectionInfo = this.getSelectionInfo();

    if (selectionInfo && selectionInfo.text) {
      // Only trigger if the selection actually changed
      if (this.lastSelection !== selectionInfo.text) {
        this.lastSelection = selectionInfo.text;

        // Call all registered listeners
        this.selectionListeners.forEach(callback => {
          try {
            callback(selectionInfo);
          } catch (error) {
            console.error('Error in selection listener:', error);
          }
        });
      }
    } else if (this.lastSelection) {
      // Selection was cleared
      this.lastSelection = null;

      // Call all registered listeners with null to indicate cleared selection
      this.selectionListeners.forEach(callback => {
        try {
          callback(null);
        } catch (error) {
          console.error('Error in selection listener:', error);
        }
      });
    }
  }

  // Get the current selection with validation
  getCurrentSelection() {
    const selectionInfo = this.getSelectionInfo();
    const validation = this.validateSelection(selectionInfo);

    return {
      ...selectionInfo,
      validation: validation
    };
  }

  // Cleanup method to remove all listeners
  cleanup() {
    this.selectionListeners = [];
    document.removeEventListener('mouseup', this.handleSelectionChange);
    document.removeEventListener('keyup', this.handleSelectionChange);
  }
}

// Create a singleton instance
export const textSelectionService = new TextSelectionService();

// Export the class for creating custom instances if needed
export default TextSelectionService;