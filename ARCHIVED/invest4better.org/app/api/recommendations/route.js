// /app/api/recommendations/route.js

import { NextResponse } from "next/server";

export async function POST(request) {
  try {
    // 1. Get the prompt from the client's request
    const { prompt } = await request.json();

    if (!prompt) {
      return NextResponse.json(
        { error: "Prompt is required." },
        { status: 400 }
      );
    }

    // 2. Securely get the API key from server-side environment variables
    const apiKey = process.env.OPENAI_API_KEY;
    if (!apiKey) {
      throw new Error("OpenAI API key is not configured on the server.");
    }

    // 3. Call the OpenAI API from your server
    const response = await fetch("https://api.openai.com/v1/chat/completions", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        Authorization: `Bearer ${apiKey}`, // The key is used securely here
      },
      body: JSON.stringify({
        model: "gpt-4o-mini",
        messages: [
          {
            role: "system",
            content:
              "You are a financial analyst specializing in ESG investing.",
          },
          { role: "user", content: prompt },
        ],
        max_tokens: 1500,
      }),
    });

    if (!response.ok) {
      const errorData = await response.json();
      console.error("OpenAI API Error:", errorData);
      throw new Error(
        `OpenAI API request failed with status ${response.status}`
      );
    }

    const data = await response.json();
    const content = data.choices?.[0]?.message?.content;

    // 4. Send the result back to the client
    return NextResponse.json({ recommendation: content });
  } catch (error) {
    console.error("Internal Server Error:", error);
    return NextResponse.json({ error: error.message }, { status: 500 });
  }
}
