# 💬 Research Chat API: AI-Powered Document Q&A 📚

## ✨ Project Overview

[ResearchChat.org](https://researchchat.org/) is an innovative tool designed to transform how users interact with complex documents like **research papers, financial reports, and congressional bills**. This project is a collaborative effort by **Abhay Gupta (John Jay High School)** and **Rishabh Nambiar (Binghamton University)**, aimed at **democratizing access to knowledge** by simplifying dense, jargon-filled text into clear, digestible answers.

This API serves as the intelligent backbone of Research Chat, showcasing an implementation that leverages cutting-edge AI to provide accurate, context-specific answers. It's designed to illustrate how such a system can simplify convoluted language and lead to more efficient insights. 🚀

---

## 💡 Skills & Technologies

This project showcases expertise in:

- **Web Development:** Next.js (API Routes)
- **AI/ML Integration:** Azure OpenAI Service (Chat Completions, Streaming API)
- **Backend Development:** Node.js, API Design
- **Language Models:** Prompt Engineering, System Message Design
- **Data Handling:** JSON Parsing, Request/Response Management
- **Error Handling:** Robust API Error Management
- **Deployment:** Azure Cloud Platform (Conceptual)
- **Languages:** JavaScript
- **Version Control:** Git, GitHub

## ✅ Key Benefits & Features

- **🔍 Concise Summaries:** Quickly access concise, AI-generated answers and summaries from intricate research papers and other documents.

- **⏱️ Time-Saving:** Spend less time deciphering jargon and more time focusing on critical insights.

- **💡 Enhanced Understanding:** Boost productivity and elevate your research by getting direct answers from your documents.

- **🔒 Context-Aware AI:** The AI strictly adheres to the provided document text and conversation history, ensuring answers are always relevant and accurate.

- **⚡ Streaming Responses:** Enjoy a fast and responsive user experience with streamed AI responses.

---

## 💻 How It Works

This Next.js API route serves as an intelligent intermediary, processing user questions against provided document content using Azure OpenAI's powerful language models.

1. **📥 Request Reception:** The API receives a `POST` request containing:

   - `fullText`: The complete text of the document to be analyzed.

   - `question`: The user's query about the document.

   - `messages`: The previous conversation history, providing crucial context for follow-up questions.

2. **🧠 AI Processing (Azure OpenAI):**

   - An **Azure OpenAI client** is initialized with predefined configurations.

   - A **system message** instructs the AI to act as an expert in answering questions about research papers, financial documents, and congressional bills. It emphasizes providing accurate, concise answers derived **STRICTLY from the provided context** and explicitly states not to use outside knowledge.

   - A **user message** combines the `fullText` of the document, the `messages` (conversation history), and the `question` into a single prompt for the AI.

   - The API calls Azure OpenAI's chat completions endpoint with the specified model (`AZURE_CONFIG.deployment`) and streams the response.

3. **📤 Streaming Response:**

   - The streamed AI response chunks are collected and encoded.

   - These chunks are then sent back to the client as a `ReadableStream`, allowing the front-end to display the AI's answer incrementally for a smooth user experience.

4. **❌ Error Handling:**

   - The API includes `try/catch` blocks to gracefully handle potential errors, such as missing `fullText` or `question`, or issues during the AI client interaction.

   - Error messages are returned in a JSON format with appropriate HTTP status codes.

---

## ⚙️ Technical Details

- **Framework:** Next.js (API Routes)

- **AI Service:** Azure OpenAI

- **Language:** JavaScript (Node.js)

- **Dependencies:**

  - `next`

  - `openai` (presumably, for `getAzureOpenAIClient` and `client.chat.completions.create`)

---

## 🚀 Implementation Showcase

This README describes an **implementation example** of an AI-powered document Q&A system. It's intended to demonstrate the capabilities and architecture used in Research Chat.
